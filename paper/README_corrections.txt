corrections to the i2mtc 2027 paper

base version 7bb89ad on branch v3.1 line numbers are from main.tex

i corrected six errors they are small usually one word or one number but each of them could lead to a wrong conclusion or give a reviewer a reason to question the results
no equation table figure or paragraph was added or removed the only measured value that changes is the jitter within a stop

maximum error and p95 (line 462)

before: errors of up to 1.5 m
after: a p95 error of up to 1.5 m
why it matters: 1.5 m is the 95th percentile the maximum error is 1.9 m so the old sentence understated the worst case

upper bound at stops d and e (line 458)

before: an upper bound on the error
after: an upper bound on the repeatability
why it matters: there is no reference at d and e and the scatter between visits cant show a constant offset so the old wording claimed an accuracy we did not measure

definition of jitter (lines 268 and 392)

before: 4.0 mm and 1.4 mm were both called jitter but came from different formulas
after: jitter is the 2d standard deviation everywhere and the value within a stop is 3.8 mm instead of 1.4 mm
why it matters: the two numbers could not be compared the conclusion stays the same since the spread between visits (1 to 9 cm) is still far larger

height filter (line 316)

before: |z| < 0.4 m around the level reported for the mobile beacon
after: |z| < 0.4 m, a band that contains the level reported for the mobile beacon at rest
why it matters: the band is centred on 0 not on 0.07 m so anyone repeating the filter from the old text would use a different band

temperature effect (lines 277 and 278)

before: a change of ±3 °C was described as having an effect of 1.8 cm
after: a rectangular distribution of ±3 °C gives a standard uncertainty of about 1.8 cm
why it matters: 1.8 cm is the standard uncertainty the effect at the ±3 °C limit is about 3.1 cm so the old sentence understated it

range of the beacon height errors (lines 257 258 368 and 470)

before: from 1 m upwards and every height error of 1 m or more
after: the range is written as -2.6 to +0.5 m and the claim reads a beacon set 1 m or more too low
why it matters: the simulation does not cover beacons set 1 m or more too high so the recommended height check was claimed for cases we did not test
