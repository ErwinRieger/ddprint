

set grid
set yrange [0:35]

# BaseTemp=190

# Startwert steigung
a=0.5

# Startwert y-achse
b=5
f(x)=b+a*(x-190)

fit f(x) "-" using 1:3 noerror via a,b
188.575356 1.479618 1.330301 0.899084
192.339678 6.284469 5.652069 0.899371
198.013471 12.588320 11.329460 0.899998
203.344344 13.652550 12.275364 0.899126
208.549293 15.041267 13.513681 0.898440
213.780295 15.394237 13.834121 0.898656
218.665050 16.381181 14.733922 0.899442
223.636731 17.665130 15.898582 0.899998
228.864726 19.477713 17.511267 0.899041
233.166611 19.965709 17.960806 0.899583
238.703829 21.434901 19.289263 0.899900
243.778700 22.389375 20.133346 0.899237
248.388788 22.741413 20.434863 0.898575
253.603245 23.424236 20.974467 0.895417
258.841566 25.044650 22.532692 0.899701
E

plot "-" using 1:2 with linespoints title "Target Flowrate", \
     "-" using 1:3 with linespoints title "Actual Flowrate", \
     "-" using 1:3 with linespoints smooth bezier title "Actual Flowrate smooth", \
     f(x) title sprintf("y=B+A*x, A=%.2f, B=%.1f, TempFactor 1/A: %.2f", a, b, 1/a)
188.575356 1.479618 1.330301 0.899084
192.339678 6.284469 5.652069 0.899371
198.013471 12.588320 11.329460 0.899998
203.344344 13.652550 12.275364 0.899126
208.549293 15.041267 13.513681 0.898440
213.780295 15.394237 13.834121 0.898656
218.665050 16.381181 14.733922 0.899442
223.636731 17.665130 15.898582 0.899998
228.864726 19.477713 17.511267 0.899041
233.166611 19.965709 17.960806 0.899583
238.703829 21.434901 19.289263 0.899900
243.778700 22.389375 20.133346 0.899237
248.388788 22.741413 20.434863 0.898575
253.603245 23.424236 20.974467 0.895417
258.841566 25.044650 22.532692 0.899701
E
188.575356 1.479618 1.330301 0.899084
192.339678 6.284469 5.652069 0.899371
198.013471 12.588320 11.329460 0.899998
203.344344 13.652550 12.275364 0.899126
208.549293 15.041267 13.513681 0.898440
213.780295 15.394237 13.834121 0.898656
218.665050 16.381181 14.733922 0.899442
223.636731 17.665130 15.898582 0.899998
228.864726 19.477713 17.511267 0.899041
233.166611 19.965709 17.960806 0.899583
238.703829 21.434901 19.289263 0.899900
243.778700 22.389375 20.133346 0.899237
248.388788 22.741413 20.434863 0.898575
253.603245 23.424236 20.974467 0.895417
258.841566 25.044650 22.532692 0.899701
E
188.575356 1.479618 1.330301 0.899084
192.339678 6.284469 5.652069 0.899371
198.013471 12.588320 11.329460 0.899998
203.344344 13.652550 12.275364 0.899126
208.549293 15.041267 13.513681 0.898440
213.780295 15.394237 13.834121 0.898656
218.665050 16.381181 14.733922 0.899442
223.636731 17.665130 15.898582 0.899998
228.864726 19.477713 17.511267 0.899041
233.166611 19.965709 17.960806 0.899583
238.703829 21.434901 19.289263 0.899900
243.778700 22.389375 20.133346 0.899237
248.388788 22.741413 20.434863 0.898575
253.603245 23.424236 20.974467 0.895417
258.841566 25.044650 22.532692 0.899701
E
pause mouse close