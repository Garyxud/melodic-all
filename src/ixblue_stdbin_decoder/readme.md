# iXblue parsing library for protocol iXblue stdbin


iXblue is a global leader in the design and manufacturing of innovative
solutions devoted to navigation, positioning and underwater imaging,
shipbuilding, as well as photonics. Using its unique in-house technology,
the company offers turnkey solutions to its Civil and Defense customers to
carry out their sea, land and space operations with optimum efficiency and
reliability. Employing a workforce of 600 people worldwide, iXblue conducts
its business with over 60 countries.


The aim of this library is to parse the iXblue standard binary protocol which is the most generic input/output protocol
for iXblue inertial system. In this library the protocol is only parsed as output protocol.


Please note that a [ROS driver](https://github.com/ixblue/ixblue_ins_stdbin_driver) allows to format the protocol data into ROS messages thanks to the parsing library.

![iXblue logo](image/ixblue_logo.jpg)

---
## Table of Contents

1. [Protocol](#protocol)
2. [Installation](#installation)
3. [Implementation](#implementation)
4. [License](#license)

---
## Protocol

The protocol iXblue stdbin allows to obtain as much information as possible from inertial system data thanks to its modularity.
Here is a non exhaustive list of data that are proposed in the protocol:

* Attitude
* Positions
* Status
* Accelerations
* ...

This information can be given in vessel or geographic frame, compensated or not from gravity and earth rotation.
You can find the details of this protocol in the interface library document of your product user manual.

**[Back to top](#table-of-contents)**

---
## Installation

### Dependencies
* [Boost](https://www.boost.org/) - Useful C++ library
* [Gtest](https://github.com/google/googletest) - Used to create the unit tests

This librairy is used in our [ROS driver](https://github.com/ixblue/ixblue_ins_stdbin_driver) that you can find
[here on Github](https://github.com/ixblue/ixblue_ins_stdbin_driver).

### Build the library

If you have not already done so, you can first download the source code in the dedicated library folder:

```sh
git clone https://github.com/ixblue/ixblue_stdbin_decoder
```

Then you can build the code in a build directory from the library folder:

```sh
cd ixblue_stdbin_decoder
mkdir build
cd build/
cmake ..
make
sudo make install
```

### Run Unit Tests

If you need to run unit tests, you can do it in your build directory by first enabling the tests:

To run all tests :

```sh
cmake -DBUILD_TESTING ..
make test
```

To run only one test :

```sh
./bin/name_of_the_test
```

**[Back to top](#table-of-contents)**

---
## Usage

This library doesn't manage the communication with the IMU. This library is just a parser. If you want, you can try our [ROS driver](https://github.com/ixblue/ixblue_ins_stdbin_driver) that implements an UDP Client based on boost asio to receive data from the IMU and publish them as ROS topic.

The entry point of this library is the class ```StdBinDecoder```.

Here is a minimal usage example:

```C++
StdBinDecoder decoder;

std::vector<uint8_t> binaryDatas;
// Fill binary data with data received from IMU
try {
  if(decoder.parse(binaryDatas)) {
    auto navDatas = decoder.getLastMessage();
  }
} catch(std::runtime_error& e) {
   // Parsing error are reported by throwing std::runtime_exception.
}
```

The ```StdBinDecoder::parse``` method accepts incomplete data (as the content of splitted TCP packet). This method returns true if the whole frame has been received, and decoded.

Parsing error are reported by throwing `std::runtime_execpetion`.

**[Back to top](#table-of-contents)**

---
## Bug repports:

Feel free to open a Github issue if you encounter any problem with this parser, we will process it as soon as possible.

---
## License

This project is licensed under the MIT License

#### (The MIT License)

Copyright (c) 2019 iXblue SAS

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
'Software'), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


**[Back to top](#table-of-contents)**

