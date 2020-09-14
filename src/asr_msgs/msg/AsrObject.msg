# Copyright (c) 2016, Kleinert Daniel, Meissner Pascal, Yi Xie
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other 
# materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific # prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#Asr object ros message. @author Pascal Meissner. @version See SVN

#Geometrical and qualitative information about a localized object with corresponding uncertainties.

#Frame relative to which poses are given and timestamp.
Header header

#The concrete localization system used to get this object.
string providedBy

#Vector of different estimates of pose of given objects. Usually containing just a single element.
#Each pose represented as multivariate normal distribution with 3D position vector and quaternion as mean and a covariance matrix.
geometry_msgs/PoseWithCovariance[] sampledPoses

#8 Corner Points of bounding box of object (part) concerning the axes of the frame of the part.
#    4-----5          z
#   /|    /|         /              x right
#  / |   / |        /               y down
# 0-----1  |       /-------x        z forwar 
# |  |  |  |       |
# |  6--|--7       |
# | /   | /        |
# |/	|/         y
# 2-----3	
geometry_msgs/Point[8] boundingBox
float64 sizeConfidence

#Misc object (part) information. Color string should be set to textured for TexturedRecognition.
string colorName
std_msgs/ColorRGBA color

#Type of the object (part).
string type
float64 typeConfidence

#Used to destinguish different entities of the same object class.
string identifier

# Reference to mesh based object model
string meshResourcePath

