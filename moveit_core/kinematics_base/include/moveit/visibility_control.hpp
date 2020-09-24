// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Open Source Robotics Foundation, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINEMATICS_BASE__VISIBILITY_CONTROL_HPP_
#define KINEMATICS_BASE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINEMATICS_BASE_EXPORT __attribute__ ((dllexport))
    #define KINEMATICS_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define KINEMATICS_BASE_EXPORT __declspec(dllexport)
    #define KINEMATICS_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINEMATICS_BASE_BUILDING_LIBRARY
    #define KINEMATICS_BASE_PUBLIC KINEMATICS_BASE_EXPORT
  #else
    #define KINEMATICS_BASE_PUBLIC KINEMATICS_BASE_IMPORT
  #endif
  #define KINEMATICS_BASE_PUBLIC_TYPE KINEMATICS_BASE_PUBLIC
  #define KINEMATICS_BASE_LOCAL
#else
  #define KINEMATICS_BASE_EXPORT __attribute__ ((visibility("default")))
  #define KINEMATICS_BASE_IMPORT
  #if __GNUC__ >= 4
    #define KINEMATICS_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define KINEMATICS_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINEMATICS_BASE_PUBLIC
    #define KINEMATICS_BASE_LOCAL
  #endif
  #define KINEMATICS_BASE_PUBLIC_TYPE
#endif

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINEMATICS_BASE_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define KINEMATICS_BASE_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define KINEMATICS_BASE_PLUGIN_EXPORT __declspec(dllexport)
    #define KINEMATICS_BASE_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINEMATICS_BASE_PLUGIN_BUILDING_LIBRARY
    #define KINEMATICS_BASE_PLUGIN_PUBLIC KINEMATICS_BASE_PLUGIN_EXPORT
  #else
    #define KINEMATICS_BASE_PLUGIN_PUBLIC KINEMATICS_BASE_PLUGIN_IMPORT
  #endif
  #define KINEMATICS_BASE_PLUGIN_PUBLIC_TYPE KINEMATICS_BASE_PLUGIN_PUBLIC
  #define KINEMATICS_BASE_PLUGIN_LOCAL
#else
  #define KINEMATICS_BASE_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define KINEMATICS_BASE_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define KINEMATICS_BASE_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define KINEMATICS_BASE_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINEMATICS_BASE_PLUGIN_PUBLIC
    #define KINEMATICS_BASE_PLUGIN_LOCAL
  #endif
  #define KINEMATICS_BASE_PLUGIN_PUBLIC_TYPE
#endif

#endif  // KINEMATICS_BASE__VISIBILITY_CONTROL_HPP_