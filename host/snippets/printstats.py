import pstats, sys


fn = 'ddprintstats.prof'
if len(sys.argv) > 1:
    fn = sys.argv[1]

p = pstats.Stats(fn)

ps = p.strip_dirs()

ps.sort_stats('tot').print_stats(100)
ps.sort_stats('cumulative').print_stats(100)
ps.sort_stats('ncall').print_stats(100)

