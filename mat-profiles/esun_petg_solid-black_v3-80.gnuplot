

set grid
set yrange [0:35]

# BaseTemp=210

# Startwert steigung
a=0.5

# Startwert y-achse
b=5
f(x)=b+a*(x-210)

fit f(x) "-" using 1:3 noerror via a,b
210.206167 4.060015 3.651166 0.899299
213.969545 4.320656 3.881665 0.898397
220.070541 4.546627 4.079117 0.897174
225.098688 4.865183 4.359256 0.896011
230.040205 5.128083 4.614858 0.899919
235.752071 5.561389 5.002460 0.899498
240.776277 6.054732 5.448258 0.899835
245.709505 7.571388 6.807416 0.899097
249.667304 8.174082 7.351377 0.899352
254.561638 11.615347 10.443188 0.899085
259.719737 12.027285 10.821269 0.899727
E

plot "-" using 1:2 with linespoints title "Target Flowrate", \
     "-" using 1:3 with linespoints title "Actual Flowrate", \
     "-" using 1:3 with linespoints smooth bezier title "Actual Flowrate smooth", \
     f(x) title sprintf("y=B+A*x, A=%.2f, B=%.1f, TempFactor 1/A: %.2f", a, b, 1/a)
210.206167 4.060015 3.651166 0.899299
213.969545 4.320656 3.881665 0.898397
220.070541 4.546627 4.079117 0.897174
225.098688 4.865183 4.359256 0.896011
230.040205 5.128083 4.614858 0.899919
235.752071 5.561389 5.002460 0.899498
240.776277 6.054732 5.448258 0.899835
245.709505 7.571388 6.807416 0.899097
249.667304 8.174082 7.351377 0.899352
254.561638 11.615347 10.443188 0.899085
259.719737 12.027285 10.821269 0.899727
E
210.206167 4.060015 3.651166 0.899299
213.969545 4.320656 3.881665 0.898397
220.070541 4.546627 4.079117 0.897174
225.098688 4.865183 4.359256 0.896011
230.040205 5.128083 4.614858 0.899919
235.752071 5.561389 5.002460 0.899498
240.776277 6.054732 5.448258 0.899835
245.709505 7.571388 6.807416 0.899097
249.667304 8.174082 7.351377 0.899352
254.561638 11.615347 10.443188 0.899085
259.719737 12.027285 10.821269 0.899727
E
210.206167 4.060015 3.651166 0.899299
213.969545 4.320656 3.881665 0.898397
220.070541 4.546627 4.079117 0.897174
225.098688 4.865183 4.359256 0.896011
230.040205 5.128083 4.614858 0.899919
235.752071 5.561389 5.002460 0.899498
240.776277 6.054732 5.448258 0.899835
245.709505 7.571388 6.807416 0.899097
249.667304 8.174082 7.351377 0.899352
254.561638 11.615347 10.443188 0.899085
259.719737 12.027285 10.821269 0.899727
E
pause mouse close