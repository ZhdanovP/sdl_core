/*
* Copyright (c) 2014, Ford Motor Company
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following
* disclaimer in the documentation and/or other materials provided with the
* distribution.
*
* Neither the name of the Ford Motor Company nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <sys/time.h>
#include <stdint.h>
#include "utils/date_time.h"


namespace date_time {

int32_t const DateTime::MILLISECONDS_IN_SECOND;
int32_t const DateTime::MICROSECONDS_IN_MILLISECONDS;

TimevalStruct DateTime::getCurrentTime() {
  TimevalStruct currentTime;
  timezone timeZone;

  gettimeofday(&currentTime, &timeZone);

  return currentTime;
}

int64_t date_time::DateTime::getSecs(const TimevalStruct &time) {
   return static_cast<int64_t>(time.tv_sec);
}

int64_t DateTime::getmSecs(const TimevalStruct &time) {
  return static_cast<int64_t>(time.tv_sec) * MILLISECONDS_IN_SECOND
      + time.tv_usec / MICROSECONDS_IN_MILLISECONDS;
}

int64_t DateTime::getuSecs(const TimevalStruct &time) {
  return static_cast<int64_t>(time.tv_sec) * MILLISECONDS_IN_SECOND
      * MICROSECONDS_IN_MILLISECONDS + time.tv_usec;
}

int64_t DateTime::calculateTimeSpan(const TimevalStruct& sinceTime) {
  return calculateTimeDiff(getCurrentTime(), sinceTime);
}

int64_t DateTime::calculateTimeDiff(const TimevalStruct &time1,
                                    const TimevalStruct &time2){
  TimevalStruct timeDifference;
  timeDifference.tv_sec = time1.tv_sec - time2.tv_sec;
  timeDifference.tv_usec = time1.tv_usec - time2.tv_usec;

  if ( timeDifference.tv_usec < 0 ) {
    timeDifference.tv_sec--;
    timeDifference.tv_usec += MILLISECONDS_IN_SECOND
                            * MICROSECONDS_IN_MILLISECONDS;
  }
  return getmSecs(timeDifference);
}

TimeCompare date_time::DateTime::compareTime(const TimevalStruct &time1, const TimevalStruct &time2) {
  if (getSecs(time1)== getSecs(time2)) {
      if (getmSecs(time1)== getmSecs(time2)) {
          if (getuSecs(time1)== getuSecs(time2)) {
            return EQUAL;
          }
          return getuSecs(time1)< getuSecs(time2)?LESS:GREATER;
      }
      return getmSecs(time1)< getmSecs(time2)?LESS:GREATER;
  }
  return getSecs(time1)< getSecs(time2)?LESS:GREATER;

}



}  // namespace date_time
