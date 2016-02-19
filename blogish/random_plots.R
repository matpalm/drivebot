W = 480
H = 320
require(ggplot2)

png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.baseline.png", width=W, height=H)
df1 = read.delim("~/dev/drivebot/src/runs/baseline/stats.tsv", h=F, col.names=c("n", "length", "reward"))
#df1 = read.delim("~/dev/drivebot/src/runs/20160129.baseline/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df1$run = as.factor('baseline')
ggplot(df1, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="baseline")
dev.off()

#df2 = read.delim("~/dev/drivebot/src/runs/nn_v4/stats.tsv", h=F, col.names=c("n", "length", "reward"))
#df2 = head(df2, 100)
#df2$run = as.factor('nn')
#ggplot(df2, aes(n, reward)) + geom_point() + ylim(0, 1000) + labs(title="nn q learnt")

png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.discrete.png", width=W, height=H)
df3 = read.delim("~/dev/drivebot/src/runs/discrete_argmax.hl1.e200/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df3$run = as.factor('discrete q table')
ggplot(df3, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="discrete q table")
dev.off()

png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.nn_argmax.png", width=W, height=H)
df4 = read.delim("~/dev/drivebot/src/runs/nn_argmax.hl1.e200/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df4$run = as.factor('nn argmax')
ggplot(df4, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="nn q table, argmax")
dev.off()

png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.nn_explore.png", width=W, height=H)
df5 = read.delim("~/dev/drivebot/src/runs/nn_run2.hl1/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df5$run = as.factor('nn_run2.hl1')
ggplot(df5, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="nn q table, explore/exploit")
dev.off()

png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.nn_replay.png", width=W, height=H)
df6 = read.delim("~/dev/drivebot/src/runs/nn_run3.replay.e200/stats.tsv", h=F, col.names=c("n", "length", "reward"))
#df5$run = as.factor('nn_run2.hl1')
ggplot(df6, aes(n, reward)) + geom_point() + ylim(0-50, 1000) + labs(title="nn q table, replay")
dev.off()

df6 = read.delim("~/dev/drivebot/src/runs/nn_run3.replay.e200.hl10/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df6 = head(df6, 200)
#df5$run = as.factor('nn_run2.hl1')
ggplot(df6, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="nn q table, replay, hl=10")

# previous run doing better
png("/home/mat/blog/blogofile/imgs/2016/drivebot/stat.nn_replay.png", width=W, height=H)
df6 = read.delim("~/dev/drivebot/src/runs/nn_v4/stats.tsv", h=F, col.names=c("n", "length", "reward"))
df6 = head(df6, 200)
df6$run = as.factor('nn replay')
ggplot(df6, aes(n, reward)) + geom_point() + ylim(-50, 1000) + labs(title="nn q table, replay")
dev.off()

png("/home/mat/blog/blogofile/imgs/2016/drivebot/boxplot_comparison.2.png", width=W, height=H)
df = rbind(df1, df3, df4) #, df6)
ggplot(df, aes(run, reward)) + geom_boxplot()
dev.off()
