/**
 * @file PlayPauseButton.qml
 * @brief Behavior of Play/Pause button.
 * Copyright (c) 2013, Ford Motor Company
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

import QtQuick 1.1

Image {
    id: playPauseButton
    property string sourceOnPressed: ""
    property string sourceOnReleased: ""

    signal clicked

    MouseArea {
        anchors.fill: parent
        onPressed: {
            playPauseButton.source = playPauseButton.sourceOnPressed
        }
        onReleased: {
            playPauseButton.source = playPauseButton.sourceOnReleased
        }
        onClicked: {
            playPauseButton.clicked()
        }
    }
    states: [
        State {
            name: "Play"
            PropertyChanges {
                target: playPauseButton
                source: "../res/buttons/player_play_btn.png"
                sourceOnPressed: "../res/buttons/player_play_pressed_btn.png"
                sourceOnReleased: "../res/buttons/player_pause_btn.png"
            }
        },

        State {
            name: "Pause"
            PropertyChanges {
                target: playPauseButton
                source: "../res/buttons/player_pause_btn.png"
                sourceOnPressed: "../res/buttons/player_pause_pressed_btn.png"
                sourceOnReleased: "../res/buttons/player_play_btn.png"
            }
        }
    ]
}
