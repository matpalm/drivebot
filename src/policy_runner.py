#!/usr/bin/env python
import argparse
from drivebot.msg import TrainingExample
from drivebot.srv import ActionGivenState
import math
import policy.baseline
import policy.discrete_q_table
import policy.nn_q_table
import rospy

parser = argparse.ArgumentParser()
parser.add_argument('--policy', type=str, default="Baseline", help="what policy to use;"\
                    " Baseline / DiscreteQTablePolicy / NNQTablePolicy")
parser.add_argument('--state-size', type=int, help="state size we expect from bots" \
                    " (dependent on their sonar to state config)")
parser.add_argument('--q-discount', type=float, default=0.9, help="q table discount." \
                    " 0 => ignore future possible rewards, 1 => assume q future rewards" \
                    " perfect. only applicable for QTablePolicies.")
parser.add_argument('--q-learning-rate', type=float, default=0.1, help="q table learning" \
                    " rate. different interp between discrete & nn policies")
parser.add_argument('--q-state-normalisation-squash', type=float, default=0.001,
                    help="what power to raise sonar ranges to before normalisation."\
                         " <1 => explore (tends to uniform)," \
                         " >1 => exploit (tends to argmax)."\
                         " only applicable for QTablePolicies.")
# nn policy specific
parser.add_argument('--gradient-clip', type=float, default=10)
parser.add_argument('--summary-log-dir', type=str, default="/tmp/nn_q_table",
                    help="where to write tensorflow summaries (for the tensorflow models)")
parser.add_argument('--summary-log-freq', type=int, default=100,
                    help="freq (in training examples) in which to write to summary")
parser.add_argument('--target-network-update-freq', type=int, default=10,
                    help="freq (in training examples) in which to flush core network to" \
                    " target network")
parser.add_argument('--target-network-update-coeff', type=float, default=1.0,
                    help="affine coeff for target network update. 0 => no update," \
                    " 0.5 => mean of core/target, 1.0 => clobber target completely")
opts = parser.parse_args()
print "OPTS", opts

# push refreshable args to param server
rospy.set_param("/q_table_policy/discount", opts.q_discount)
rospy.set_param("/q_table_policy/learning_rate", opts.q_learning_rate)
rospy.set_param("/q_table_policy/state_normalisation_squash",
                opts.q_state_normalisation_squash)
rospy.set_param("/q_table_policy/summary_log_freq", opts.summary_log_freq)
rospy.set_param("/q_table_policy/target_network_update_freq",
                opts.target_network_update_freq)

# build policy
NUM_ACTIONS = 4  # TODO: shared with sim
if opts.policy == "Baseline":
    policy = policy.baseline.BaselinePolicy()
elif opts.policy == "DiscreteQTablePolicy":
    policy = policy.discrete_q_table.DiscreteQTablePolicy(num_actions=NUM_ACTIONS)
elif opts.policy == "NNQTablePolicy":
    hidden_size = int(math.sqrt(opts.state_size * NUM_ACTIONS))
    print "NNQTablePolicy #input", opts.state_size, "#hidden", hidden_size
    policy = policy.nn_q_table.NNQTablePolicy(state_size=opts.state_size,
                        num_actions=NUM_ACTIONS, hidden_layer_size=hidden_size,
                        gradient_clip=opts.gradient_clip,
                        target_network_update_coeff=opts.target_network_update_coeff,
                        summary_file=opts.summary_log_dir)
else:
    raise Exception("unknown --policy %s" % opts.policy)

# wire training_egs topic to policy
def call_policy_training(eg):
    policy.train(eg.state1, eg.action, eg.reward, eg.state2)
rospy.Subscriber('/drivebot/training_egs', TrainingExample, call_policy_training)

# proxy action_given_state call to policy
def call_policy_action_given_state(ags_req):
    return policy.action_given_state(ags_req.state)
service = rospy.Service("/drivebot/action_given_state", ActionGivenState,
                        call_policy_action_given_state)

# run
rospy.init_node('policy_runner')
rospy.spin()

