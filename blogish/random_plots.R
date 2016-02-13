require(ggplot2)

df1 = read.delim("~/dev/drivebot/src/runs/20160129.baseline/stats.tsv", col.names=c("n", "length", "reward"))
#df1 = head(df1, 100)
df1$run = as.factor('baseline')
ggplot(df1, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="baseline")

#df2 = read.delim("~/dev/drivebot/src/runs/nn_v4/stats.tsv", col.names=c("n", "length", "reward"))
#df2 = head(df2, 100)
#df2$run = as.factor('nn')
#ggplot(df2, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="nn q learnt")

df3 = read.delim("~/dev/drivebot/src/runs/discrete_argmax.hl1/stats.tsv", col.names=c("n", "length", "reward"))
df3$run = as.factor('q_table.argmax.hl1')
ggplot(df3, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="discrete q table, argmax")
summary(df3)

df4 = read.delim("~/dev/drivebot/src/runs/nn_argmax.hl1/stats.tsv", col.names=c("n", "length", "reward"))
df4$run = as.factor('nn.argmax.hl1')
ggplot(df4, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="nn q table, argmax")

df5 = read.delim("~/dev/drivebot/src/runs/nn_run3.replay/stats.tsv", col.names=c("n", "length", "reward"))
df5$run = as.factor('nn_run2.hl1')
ggplot(df5, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="nn q table, rampup")


df = rbind(df1, df2, df3)
summary(df)

ggplot(df, aes(run, reward)) + geom_boxplot()