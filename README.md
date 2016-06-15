# workshop-extended-example
Extended template for package with two components.

### Usage:
```bash
cd to_your_workspace
git clone https://github.com/cogimon/workshop-extended-example.git
cd workshop-extended-example
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=/vol/cogimon/cogimon-minimal-nightly ..
make
```
this should compile and create a package/library like:
```bash
orocos
└── gnulinux
    └── RttExamples
        └── libRttExamples-gnulinux.so
```
happy coding!
