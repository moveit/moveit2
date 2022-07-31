// Copyright 2022 PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

namespace moveit_servo
{
/**
 * Detect if realtime kernel is present.
 * \returns true if realtime kernel is detected
 */
bool has_realtime_kernel();

/**
 * Configure SCHED_FIFO thread priority for the thread that calls this function
 * \param[in] priority the priority of this thread from 0-99
 * \returns true if configuring scheduler succeeded
 */
bool configure_sched_fifo(int priority);

}  // namespace moveit_servo
