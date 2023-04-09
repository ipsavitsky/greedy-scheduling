# Code section

## Build

```bash
mkdir build
cd build
cmake ..
make
```

## Build docs

```bash
doxygen Doxyfile
```

## All dependencies

1. `graph_partitioning`
1. `json_dumper`
    1. `nlohmann/json` - bundeled
    1. `boost/lexical_cast`
1. `logging`
    1. `boost/log`
1. `parser`
    1. `boost/numeric`
1. `schedule`
    1. `boost/numeric`
    1. `boost/graph`
    1. `boost/property_map`
1. `time_schedule`
    1. `boost/numeric`
    1. `boost/iterator`
    1. `boost/range`
1. `main.cpp`
    1. `boost/program_options`
    1. `boost/lexical_cast`

## How to use it

```bash
General options:
  -h [ --help ]                 Print help
  -i [ --input ] arg            Input file (or directory)
  --criteria arg (=NO)          Extra criteria for time schedule (CR/BF/NO)
  --class arg                   Class of input file (0/1/2)
  -l [ --log ] arg (=debug)     Specify log level 
                                0 - trace 
                                1 - debug 
                                2 - info 
                                3 - warning 
                                4 - error 
                                5 - fatal 
                                
  -s [ --scheme ] arg (=access) GC2 scheme to use (simple/access)
  -t [ --threshold ] arg (=0.5) Threshold for access scheme
  --steps                       Dump steps as individual files
  --cr_con                      Enable CR controlling GC1 at 0.3 threshold


```
