from collections import Counter
from drivebot.msg import TrainingExample
import numpy as np
import rospy
import states
import tensorflow as tf
import util as u

def flatten(state):
    return np.asarray(state).reshape(1, -1)

def one_hot_1d(idx, n):
    # hack!
    oh = [0.0] * n
    oh[idx] = 1.0
    return [oh]

# build a (grouped) copy op that copies the values of all variables between two namespaces.
# use an affine_coefficient to denote the amount copied.
# target = affine_coefficient * src  +  (1.0-affine_coefficient * target)
# affine_coefficient = 0.0 => noop
# affine_coefficient = 0.5 => average
# affine_coefficient = 1.0 => totally clobber 'target' with 'src'
def copy_all_vars(from_namespace, to_namespace, affine_coefficient=1.0):
    assert affine_coefficient >= 0.0 and affine_coefficient <= 1.0
    copy_ops = []
    with tf.variable_scope("", reuse=True):  # for grabbing the targets by full namespace
        for src_var in tf.all_variables():
            # ignore any variable not in src namespace
            if not src_var.name.startswith(from_namespace):
                continue
            # fetch reference to target variable with the same name as the src variable
            assert src_var.name.endswith(":0")
            target_var_name = src_var.name.replace(from_namespace, to_namespace).replace(":0", "")
            target_var = tf.get_variable(target_var_name, src_var.get_shape())
            # create a copy op to clobber target with src
            # target = alpha * src + (1.0-alpha) * target
            copy_ops.append(target_var.assign_sub(affine_coefficient * (target_var - src_var)))
    single_copy_op = tf.group(*copy_ops)
    return single_copy_op

def mlp_layer(namespace, input, input_size, output_size, include_non_linearity=False):
    with tf.variable_scope(namespace):
        projection = tf.get_variable("projection", [input_size, output_size])
        bias = tf.get_variable("bias", [1, output_size], initializer=tf.constant_initializer(0.0))
        output = tf.matmul(input, projection) + bias
        return tf.nn.sigmoid(output) if include_non_linearity else output

def build_model(namespace, state_size, num_actions, hidden_layer_size):
    # input is a sequence of 5 * 3 readings; 5 for last 5 in history, 3 for readings (F, L, R)
    # (i.e. they are just concatted for this version as opposed to treated as a seqeucen)
    input_state = tf.placeholder(dtype = tf.float32, shape = [None, state_size])
    with tf.variable_scope(namespace):
        hidden = mlp_layer("h1", input_state, state_size, hidden_layer_size, include_non_linearity=True)
        model = mlp_layer("out", hidden, hidden_layer_size, num_actions, include_non_linearity=False)
    return input_state, model


# simple single hidden layer neural net for regressing q value for 3 actions
# based on last 5 sonar readings
# input is 15 element
class NNQTablePolicy(object):

    def __init__(self, state_size, num_actions, hidden_layer_size, gradient_clip, target_network_update_coeff, summary_file):
        self.refreshable_params_inited = False
        self.refresh_params()
        self.state_size = state_size
        self.num_actions = num_actions
        self.gradient_clip = gradient_clip
        self.target_network_update_coeff = target_network_update_coeff
        with tf.device("/cpu:0"):
            self.setup_models(hidden_layer_size, summary_file)
        rospy.Subscriber('/drivebot/training_egs', TrainingExample, self.training_msg_callback)
        self.episode_stats = Counter()
        self.calls_to_train = 0

    def refresh_params(self):
        if not self.refreshable_params_inited:
            with tf.variable_scope("refreshable_params"):
                self.discount = tf.Variable(0.9, "discount")
                self.learning_rate = tf.Variable(0.01, "learning_rate")
            self.refreshable_params_inited = True
        params = rospy.get_param("q_table_policy")
        print "REFRESH_PARAM\t%s" % params        
        self.discount.assign(params['discount'])
        self.learning_rate.assign(params['learning_rate'])
        self.state_normalisation_squash = params['state_normalisation_squash']
        self.summary_log_freq = params['summary_log_freq']
        self.target_network_update_freq = params['target_network_update_freq']

    def setup_models(self, hidden_layer_size, summary_file):
        # setup the seperate core and target networks
        self.core_state, self.core_q_values = build_model("core", self.state_size, self.num_actions, hidden_layer_size)
        self.target_state, self.target_q_values = build_model("target", self.state_size, self.num_actions, hidden_layer_size)

        # build the global copy op that will copy core network onto target
        self.clobber_target_net_op = copy_all_vars(from_namespace="core", to_namespace="target",
                                                   affine_coefficient=self.target_network_update_coeff)

        # left hand side of the bellman update; Q(s1, a)
        self.core_action_mask = tf.placeholder(dtype=tf.float32, shape=[None, self.num_actions])  # one hot mask over actions
        self.core_q_value_for_action = tf.reduce_sum(self.core_q_values * self.core_action_mask)

        # right hand side of bellman update; reward + max_a Q(s2, a')
        self.reward = tf.placeholder(dtype=tf.float32)
        self.max_target_q_value_plus_reward = self.reward + (self.discount * tf.stop_gradient(tf.reduce_max(self.target_q_values)))

        # for loss just use squared loss on the difference
        self.temporal_difference_loss = tf.reduce_mean(tf.pow(self.max_target_q_value_plus_reward - self.core_q_value_for_action, 2))
        optimizer = tf.train.GradientDescentOptimizer(self.learning_rate)
        #optimizer = tf.train.RMSPropOptimizer(learning_rate=self.learning_rate, decay=0.9)
        gradients = optimizer.compute_gradients(self.temporal_difference_loss)
        for i, (gradient, variable) in enumerate(gradients):
            if gradient is None:  # eg stop gradient cases
                continue
            gradients[i] = (tf.clip_by_norm(gradient, self.gradient_clip), variable)
            tf.histogram_summary(variable.name, variable)
            tf.histogram_summary(variable.name + '/gradients', gradient)
        tf.scalar_summary("temporal_difference_loss", self.temporal_difference_loss)
        self.train_op = optimizer.apply_gradients(gradients)

        # build session
        self.sess = tf.Session()
        self.sess.run(tf.initialize_all_variables())
        self.summaries = tf.merge_all_summaries()
        self.summary_writer = tf.train.SummaryWriter(summary_file, self.sess.graph_def)

    def action_given_state(self, state):
        state = flatten(state)
        q_values = self.sess.run(self.core_q_values, feed_dict={self.core_state: state})
        normed = u.normalised(u.raised(q_values[0], self.state_normalisation_squash))
        action = u.weighted_choice(normed)
        print "CHOOSE\t based on state", state, "q_values", q_values, "(normed to", normed, ") => action", action
        return action

    def training_msg_callback(self, eg):
        self.episode_stats['callback_training_eg'] += 1
        self.train(eg.state1, eg.action, eg.reward, eg.state2)

    def train(self, state_1, action, reward, state_2):
        self.episode_stats['>train'] += 1
        self.calls_to_train += 1
        self.episode_stats["train a %s r %s" % (action, reward)] += 1

        state_1 = flatten(state_1)
        state_2 = flatten(state_2)


        # >>> DEBUG
#        print "core_q_values BEFORE", self.sess.run(self.core_q_values, feed_dict={self.core_state: state_1})
#        print "target_q_values", self.sess.run(self.target_q_values, feed_dict={self.target_state: state_2})
#        print "max_target_q_value_plus_reward", self.sess.run(self.max_target_q_value_plus_reward,
#                                                              feed_dict={self.reward: reward,
#                                                                         self.target_state: state_2})
#        print "core_q_value_for_action 0", self.sess.run(self.core_q_value_for_action,
#                                                            feed_dict={self.core_action_mask: one_hot_1d(0, 2),
#                                                                       self.core_state: state_1})
#        print "core_q_value_for_action 1", self.sess.run(self.core_q_value_for_action,
#                                                            feed_dict={self.core_action_mask: one_hot_1d(1, 2),
#                                                                        self.core_state: state_1})
#        print "temporal_difference_loss; s0", self.sess.run(self.temporal_difference_loss,
#                                                        feed_dict={self.core_action_mask: one_hot_1d(0, 2),
#                                                                   self.core_state: state_1,
#                                                                   self.reward: reward,
#                                                                   self.target_state: state_2})
#        print "temporal_difference_loss; s1", self.sess.run(self.temporal_difference_loss,
#                                                        feed_dict={self.core_action_mask: one_hot_1d(1, 2),
#                                                                   self.core_state: state_1,
#                                                                   self.reward: reward,
#                                                                   self.target_state: state_2})
        # <<< DEBUG


        # train against temporal difference. write summaries every 100th call
        training_feed_dict = {self.core_state: state_1,
                              self.core_action_mask: one_hot_1d(action, self.num_actions),
                              self.reward: reward,
                              self.target_state: state_2}
        if self.calls_to_train % self.summary_log_freq == 0:
            _opt, summaries = self.sess.run([self.train_op, self.summaries], feed_dict=training_feed_dict)
            self.summary_writer.add_summary(summaries, self.calls_to_train)
        else:
            _opt = self.sess.run(self.train_op, feed_dict=training_feed_dict)

#        print "core_q_values AFTER", self.sess.run(self.core_q_values, feed_dict={self.core_state: state_1})

        # copy across target network from time to time
        if self.calls_to_train % self.target_network_update_freq == 0:
            self.sess.run(self.clobber_target_net_op)

    def debug_model(self):
        pass
        #TODO:

    def end_of_episode(self):
        print ">>> end of episode stats"
        self.refresh_params()
        print "EPISODE STATS", self.episode_stats
        self.episode_stats = Counter()




