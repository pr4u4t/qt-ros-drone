import QtMultimedia 5.12
import QtQuick 2.12
import QtQuick.Window 2.12 as W
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4
import QtQuick.Extras.Private 1.0
import QtSensors 5.10
import QtQml 2.12
import graph 1.0

//import QtQuick.Scene3D 2.15
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

//import Ros 1.0

W.Window {
    id: main
    visible: true
    width: 1920
    height: 1080
    color: "black"

    title: qsTr("Gazebo")

    property bool mShowState: true





  //  Image {
    //    id: imgBackground
 //       anchors.fill: parent
   //     opacity: 0.5
 //       source: "image://roscimage/usb_cam/image_raw"
//    }

    StackLayout {

        id: stack
        anchors.fill: parent
        currentIndex: 0

        Item {
            Image {
                id: roscimg
                visible: true
                cache: false
                anchors.fill: parent
                source: "image://roscimage/usb_cam/image_raw"

                Timer {
                    interval: 50
                    repeat: true
                    running: true
                    onTriggered: { roscimg.source = ""; roscimg.source = "image://roscimage/usb_cam/image_raw" }
                }
            }

            Image {
                id:rosimg
                visible:false
                cache: false
                anchors.fill: parent
                source: "image://rosimage/usb_cam/image_raw"

                Timer {
                    interval: 50
                    repeat: true
                    running: true
                    onTriggered: { rosimg.source = ""; rosimg.source = "image://rosimage/usb_cam/image_raw" }
                }
            }
            Text {
                id: camlowquality
                visible: true
                text: "CAM QUALITY:LOW"
                x:1150
                y:600

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        camlowquality.visible= false
                        camhighquality.visible= true
                        roscimg.visible= false
                        rosimg.visible= true
                    }
                }
            }

            Text{
                id: camhighquality
                visible: false
                text: "CAM QUALITY:HIGH"
                color: "blue"
                x:1150
                y:600

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        camlowquality.visible= true
                        camhighquality.visible= false
                        roscimg.visible= true
                        rosimg.visible= false
                    }
                }
            }

            Entity {
                id: sceneRoot

                Camera {
                    id: camera
                    projectionType: CameraLens.PerspectiveProjection
                    position: Qt.vector3d( 0.0, 0.0, 20 )
                }

                components: [
                    RenderSettings {
                        activeFrameGraph: ForwardRenderer {
                            camera: camera
                            clearColor: "white"
                        }
                        pickingSettings.pickMethod: PickingSettings.TrianglePicking
                    }

                ]

                Entity {
                    id: cube
                    components: [cubeTransform]

                    Transform {
                        id: cubeTransform
                        translation: Qt.vector3d(2, 0, 10)
                        scale3D: Qt.vector3d(1, 4, 1)
                        rotation: fromEulerAngles( roll.roll,
                                                  pitch.pitch,
                                                  yaw.yaw )
                    }
                }
            }

            Image {
                id: dialRPM
                x: mShowState ? 10 : -width
                y: 70
                width: implicitWidth * mScale
                fillMode: Image.PreserveAspectFit
                source: "qrc:/rpm.png"

                property real mScale: 0.5

                Behavior on x {
                    NumberAnimation {
                        duration: 300
                    }
                }

                CircularGauge {
                    id: gaugeRPM
                    width: 262 * dialRPM.mScale; height: width
                    x: 150 * dialRPM.mScale - width / 2; y: 155 * dialRPM.mScale - width / 2
                    value: rpmdata.rpmdata
                    maximumValue : 5000
                    stepSize: 50
                    style: CircularGaugeStyle {
                         id: gaugeRPMStyle
                         labelStepSize: 500

                         function degreesToRadians(degrees) {
                             return degrees * (Math.PI / 180);
                         }

                         tickmark: Rectangle {
                             visible: styleData.value % 500 == 0
                             implicitWidth: outerRadius * 0.02
                             antialiasing: true
                             implicitHeight: outerRadius * 0.06
                             color: "#01baff"
                         }

                         minorTickmark: Rectangle {
                             visible: false
                             implicitWidth: outerRadius * 0.01
                             antialiasing: true
                             implicitHeight: outerRadius * 0.03
                             color: "#01baff"
                         }

                         tickmarkLabel: Text {
                             font { pixelSize: 20 * dialRPM.mScale }
                             text: styleData.value
                             color: "#01baff"
                             antialiasing: true
                         }

                         needle: Rectangle {
                             y: outerRadius * 0.15
                             implicitWidth: outerRadius * 0.03
                             implicitHeight: outerRadius * 0.9
                             antialiasing: true
                             color: "#ff0000"
                         }

                         foreground: Item {
                             Image {
                                 width: implicitWidth * dialRPM.mScale
                                 fillMode: Image.PreserveAspectFit
                                 source: "qrc:/rpm_center.png"
                                 anchors.centerIn: parent
                             }
                         }

                         background: Rectangle {
                             radius: width / 2
                             color: "#000000"
                         }
                     }
                }

                Text {
                    text: "RPM: " + toFixed(rpmdata.rpmdata, 2)
                    anchors {
                        centerIn: gaugeRPM
                        verticalCenterOffset: -40 * dialRPM.mScale
                    }
                    font { pixelSize: 10 }
                    color: "#ff0000"
                }

                Item {
                    id: meterThrottle
                    anchors {
                        left: parent.left; leftMargin: 268 * dialRPM.mScale
                        bottom: parent.bottom; bottomMargin: 52 * dialRPM.mScale
                    }
                    width: imageThrottle.width
                    height: imageThrottle.height * throttledata.throttledata /200
                    clip: true

                    Image {
                        id: imageThrottle
                        anchors {
                            bottom: parent.bottom
                        }
                        width: implicitWidth * dialRPM.mScale
                        fillMode: Image.PreserveAspectFit
                        source: "qrc:/throttle.png"
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        /*
                        tachometer.visible = false
                        speedometer.visible = false
                        compassdial.visible = false
                        altimeterback.visible = false
                        ammeter.visible = false
                        //voltmeter.visible = false
                        throttlegauge.visible = false
                        batteryindicator.visible = false
                        dashboard.visible = true
                        */
                    }
                }
            }

            Rectangle {
                id: speedometer
                x: mShowState ? 10 : -width
                y: 260
                width: 170
                height: 170
                radius: width*0.5
                color: "black"

                Behavior on x {
                    NumberAnimation {
                        duration: 300
                    }
                }

                CircularGauge {
                    value: speed.speed
                    maximumValue : 10
                    x:10
                    y:10
                    width: 170
                    height: 170
                    anchors.centerIn: parent
                    style: CircularGaugeStyle {
                        labelStepSize: 0.5
                        tickmarkStepSize: 0.5
                    }

                    Text {
                        text: "SPEED: " + Math.round(speed.speed)
                        anchors {
                            centerIn: parent
                            verticalCenterOffset: -20
                        }
                        font { pixelSize: 11 }
                        color: "white"
                    }

                    Behavior on value {
                        NumberAnimation {
                            duration: 1000
                        }
                    }

                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            /*
                            tachometer.visible = false
                            speedometer.visible = false
                            compassdial.visible = false
                            altimeterback.visible = false
                            ammeter.visible = false
                            //voltmeter.visible = false
                            throttlegauge.visible = false
                            batteryindicator.visible = false
                            dashboard.visible = true
                            */
                        }
                    }
                }
            }

            Item {
                id: dialCurrent
                x: mShowState ? 10 : -width
                y: 460
                width: 487 * mScale; height: 487 * mScale
                //fillMode: Image.PreserveAspectFit
                //source: "qrc:/current.png"

                Behavior on x {
                    NumberAnimation {
                        duration: 300
                    }
                }

                property real mScale: 0.35

                CircularGauge {
                    id: gaugeCurrent
                    anchors.fill: parent
                    value: currentdata.currentdata * 10
                    maximumValue : 100
                    stepSize: 1
                    style: CircularGaugeStyle {
                         id: gaugeCurrentStyle
                         labelStepSize: 10

                         function degreesToRadians(degrees) {
                             return degrees * (Math.PI / 180);
                         }

                         tickmark: Rectangle {
                             visible: styleData.value % 10 == 0
                             implicitWidth: outerRadius * 0.02
                             antialiasing: true
                             implicitHeight: outerRadius * 0.06
                             color: "#01baff"
                         }

                         minorTickmark: Rectangle {
                             visible: styleData.value % 10 != 0
                             implicitWidth: outerRadius * 0.01
                             antialiasing: true
                             implicitHeight: outerRadius * 0.03
                             color: "#01baff"
                         }

                         tickmarkLabel: Text {
                             font { pixelSize: 30 * dialCurrent.mScale; bold: true }
                             text: styleData.value / 10
                             color: "#01baff"
                             antialiasing: true
                         }

                         needle: Rectangle {
                             y: outerRadius * 0.15
                             implicitWidth: outerRadius * 0.03
                             implicitHeight: outerRadius * 0.9
                             antialiasing: true
                             color: "#01baff"
                         }

                         foreground: Item {
                             Image {
                                 width: implicitWidth * dialCurrent.mScale
                                 fillMode: Image.PreserveAspectFit
                                 source: "qrc:/current_center.png"
                                 anchors.centerIn: parent
                             }
                         }

                         background: Rectangle {
                             radius: width / 2
                             color: "#081831"
                         }
                     }
                }

                Text {
                    text: "CURRENT: " + toFixed(currentdata.currentdata, 2)
                    anchors {
                        centerIn: parent
                        verticalCenterOffset: -60 * dialCurrent.mScale
                    }
                    font { pixelSize: 10 }
                    color: "#00ade1"
                }

                Image {
                    anchors {
                        horizontalCenter: parent.horizontalCenter
                        bottom: parent.bottom; bottomMargin: 22 * dialCurrent.mScale
                    }
                    width: implicitWidth * dialCurrent.mScale
                    fillMode: Image.PreserveAspectFit
                    source: "qrc:/current_bottom.png"

                    Image {
                        id: imgBattery1
                        anchors {
                            horizontalCenter: parent.horizontalCenter
                            bottom: parent.bottom; bottomMargin: 20 * dialCurrent.mScale
                        }
                        width: 100 * dialCurrent.mScale
                        fillMode: Image.PreserveAspectFit
                        source: "qrc:/battery1.png"

                        Text {
                            text: Math.round(battery.battery) + "%"
                            anchors {
                                centerIn: parent
                            }
                            font { pixelSize: 25 * dialCurrent.mScale }
                            color: "#0ca30e"
                        }
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                       /*
                       tachometer.visible = false
                       speedometer.visible = false
                       compassdial.visible = false
                       altimeterback.visible = false
                       ammeter.visible = false
                       //voltmeter.visible = false
                       throttlegauge.visible = false
                       batteryindicator.visible = false
                       dashboard.visible = true
                       */
                    }
                }
            }


            Image {
                id: compassdial
                source: "qrc:/compass.png"
                x: mShowState ? 1130 : 1200 + width
                y: 70
                width: 160
                height: 160

                Behavior on x {
                    NumberAnimation {
                        duration: 300
                    }
                }

                Image {
                    id: compassarrow
                    source: "ARROW-Bright.png"
                    anchors.centerIn: parent
                    width: 10
                    height: 140
                    rotation: yaw.yaw
                    //Behavior on rotation {SmoothedAnimation{velocity: 200}}
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        /*
                        tachometer.visible = false
                        speedometer.visible = false
                        compassdial.visible = false
                        altimeterback.visible = false
                        ammeter.visible = false
                        //voltmeter.visible = false
                        throttlegauge.visible = false
                        batteryindicator.visible = false
                        dashboard.visible = true
                        */
                    }

                }
            }

            Item {
                id: dialAdi
                x: mShowState ? 1130 : 1200 + width
                y: 260
                width: 240 * mScale
                height: 240 * mScale
                transformOrigin: Item.Center
                rotation: -m_roll

                property real mScale: 0.7

                property real _roll: roll.roll
                on_RollChanged: {
                    updateValues()
                }

                property real _pitch: pitch.pitch
                on_PitchChanged: {
                    updateValues()
                }

                property real m_originalPixPerDeg: 0.5

                property real m_roll: 0
                property real m_pitch: 0
                property real m_faceDeltaX_old: 0
                property real m_faceDeltaY_old: 0
                property real m_faceDeltaX_new: 0
                property real m_faceDeltaY_new: 0

                property var updateValues: function() {
                    m_faceDeltaX_old = m_faceDeltaX_new
                    m_faceDeltaY_old = m_faceDeltaY_new

                    m_roll = Math.max(-180, Math.min(180, _roll))
                    m_pitch = Math.max(-50, Math.min(50, _pitch))

                    var delta = m_originalPixPerDeg * m_pitch
                    var roll_rad = Math.PI * m_roll / 180.0
                    m_faceDeltaX_new = dialAdi.mScale * delta * Math.sin( roll_rad )
                    m_faceDeltaY_new = dialAdi.mScale * delta * Math.cos( roll_rad )

                    imgFace.x += m_faceDeltaX_new - m_faceDeltaX_old
                    imgFace.y += m_faceDeltaY_new - m_faceDeltaY_old

                    var ret = new Object

                    return ret
                }

                Image {
                    id: imgBack
                    width: implicitWidth * dialAdi.mScale
                    fillMode: Image.PreserveAspectFit
                    anchors.centerIn: parent
                    source: "qrc:/adi_back.svg"
                }

                Image {
                    id: imgFace
                    width: implicitWidth * dialAdi.mScale
                    fillMode: Image.PreserveAspectFit
                    x: parent.width / 2 - width / 2; y: parent.height / 2 - height / 2
                    source: "qrc:/adi_face.svg"
                }

                Image {
                    id: imgRing
                    width: implicitWidth * dialAdi.mScale
                    fillMode: Image.PreserveAspectFit
                    anchors.centerIn: parent
                    source: "qrc:/adi_ring.svg"
                }

                Behavior on x {
                    NumberAnimation {
                        duration: 300
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {

                    }

                }
            }

            Image {
                id: imgCase
                width: implicitWidth * dialAdi.mScale
                fillMode: Image.PreserveAspectFit
                anchors.centerIn: dialAdi
                source: "qrc:/adi_case.svg"
            }

            Item {
                id: recBallastLoop
                anchors {
                   horizontalCenter: parent.horizontalCenter
                   bottom: parent.bottom; bottomMargin: mShowState ? 0 : -height

                   Behavior on bottomMargin {
                       NumberAnimation {
                           duration: 300
                       }
                   }
                }
                width: parent.width - 500
                height: 250

                Image {
                   id: imgBallast
                   anchors {
                       left: parent.left; right: parent.right
                       verticalCenter: parent.verticalCenter
                   }
                   source: "qrc:/ballast.png"
                   fillMode: Image.PreserveAspectFit

                   property real ratio: width / implicitWidth
                }

                Image {
                   id: imgSensor1
                   anchors {
                       bottom: imgBallast.bottom; bottomMargin: 36 * imgBallast.ratio
                       left: imgBallast.left
                   }
                   width: implicitWidth * imgBallast.ratio
                   fillMode: Image.PreserveAspectFit
                   source: "qrc:/sensor1_%1.png".arg(outletpumpdata.outletpumpdata > 50 ? "green" : "red")
                }

                Image {
                   id: imgSensor2
                   anchors {
                       bottom: imgBallast.bottom; bottomMargin: 38 * imgBallast.ratio
                       right: imgBallast.right
                   }
                   width: implicitWidth * imgBallast.ratio
                   fillMode: Image.PreserveAspectFit
                   source: "qrc:/sensor2_%1.png".arg(inletpumpdata.inletpumpdata > 50 ? "green" : "red")
                }

                Image {
                   id: imgV1
                   anchors {
                       bottom: imgBallast.bottom; bottomMargin: 75 * imgBallast.ratio
                       left: imgBallast.left; leftMargin: 300 * imgBallast.ratio
                   }
                   width: implicitWidth * imgBallast.ratio
                   fillMode: Image.PreserveAspectFit
                   source: "qrc:/pump1_%1.png".arg(inflowdata.inflowdata > 0 ? "green" : "red")
                }

                Image {
                   id: imgV2
                   anchors {
                       bottom: imgBallast.bottom; bottomMargin: 124 * imgBallast.ratio
                       right: imgBallast.right; rightMargin: 348 * imgBallast.ratio
                   }
                   width: implicitWidth * imgBallast.ratio
                   fillMode: Image.PreserveAspectFit
                   source: "qrc:/pump2_%1.png".arg(outflowdata.outflowdata > 0 ? "green" : "red")
                }

                Rectangle {
                   id: line1
                   width: 3
                   anchors {
                       left: imgBallast.horizontalCenter
                       top: imgBallast.bottom; bottom: txtPressureBallast.verticalCenter
                   }
                   border { color: "#dfdfdf"; width: 2}
                }

                Rectangle {
                   id: line2
                   height: 3
                   anchors {
                       left: line1.right; right: txtPressureBallast.left
                       verticalCenter: txtPressureBallast.verticalCenter
                   }
                   border { color: "#dfdfdf"; width: 2}
                }

                Text {
                   id: txtPressureBallast
                   anchors {
                       verticalCenter: imgBallast.verticalCenter
                       horizontalCenter: imgBallast.horizontalCenter; horizontalCenterOffset: -10 * imgBallast.ratio
                   }
                   font { pixelSize: 15; bold: true }
                   text: toFixed(pressuredata.pressuredata, 2)
                   //text: outletpumpdata.outletpumpdata
                }
            }

        }

        Item {
            id: graphPage
            width: 1920
            height: 1080
            Column {

                anchors.fill: parent
                Rectangle {
                    height: dashboard.height
                    width: parent.width
                }
                Rectangle {
                    height: 500
                     width: parent.width
                    color: "green"
                }

                Rectangle {
                    property int graphHeight: 200
                    id: graphContainer
                    width: 1920
                    height: graphHeight
                    color: "black"
                    clip:true
                    Column {
                        id: col
                        z: 3
                        Row {
                            id: xRow
                            x: 10
                            Button {
                                width: 30
                                height: 30
                                text: "-"
                                onClicked: {
                                    graph.scaleDownX()
                                }
                            }
                            Button {
                                width: 30
                                height: 30
                                text: "+"
                                onClicked: {
                                    graph.scaleUpX()
                                }
                            }
                            Text {
                                text: "X scale:" + graph.xScale
                                anchors.verticalCenter: xRow.verticalCenter
                                width: 90
                            }
                        }
                        Row {
                            id: yRow
                            x: 10
                            Button {
                                width: 30
                                height: 30
                                text: "-"
                                onClicked: {
                                    graph.scaleDownY()
                                }
                            }
                            Button {
                                width: 30
                                height: 30
                                text: "+"
                                onClicked: {
                                    graph.scaleUpY()
                                }
                            }
                            Text {
                                text: "Y scale:" + graph.yScale
                                anchors.verticalCenter: yRow.verticalCenter
                                width: 90
                            }

                        }
                        Row {
                            id: holdRow
                            x: 10
                            Button {
                                width: 30
                                height: 30
                                text: "-"
                                onClicked: {
                                    graph.decreaseHoldTime()
                                }
                            }
                            Button {
                                width: 30
                                height: 30
                                text: "+"
                                onClicked: {
                                    graph.increaseHoldTime()
                                }
                            }
                            Text {
                                text: "Hold time:" +graph.holdTime + "s"
                                anchors.verticalCenter: holdRow.verticalCenter
                                width: 90
                            }
                        }
                    }

                    Flickable {
                        id: content
                        anchors.fill: parent
                        contentWidth: graph.xScale > 1 ? parent.width * graph.xScale : parent.width
                        contentHeight: graph.yScale > 1 ? graphContainer.graphHeight * graph.yScale : graphContainer.graphHeight

                        /// This is custom graph control Element. It is configured to store plot in range
                        /// from 0 to 10 and removes old data after 20 seconds.
                        Graph {
                            id: graph
                            width: content.contentWidth
                            height: content.contentHeight
                            yMin: -800 // Change this value to adjust y axis range
                            yMax: 500 // Change this value to adjust y axis range

                            clip:true
                            /// Demo function - can be removed after desired source of data is implemented
                            function newSample(i) {
                                return (Math.sin(10*(i / 100.0 * Math.PI * 2)) + 1) * 5 + Math.random() * 0.05;
                            }
                            /// Demo function - can be removed after desired source of data is implemented
//                            Component.onCompleted: {
//                                for (var i=0; i<100; ++i)
//                                    appendSample(newSample(i));
//                            }
                            property var holdTimes: [10, 20, 30, 40, 50, 60, 120, 180, 240, 300,600,1000,2000,4000,8000]
                            property int currentHoldIndex: 1
                            holdTime: holdTimes[currentHoldIndex] // Change this value to adjust flushing time (value in seconds)

                            function increaseHoldTime() {
                                if (currentHoldIndex < holdTimes.length - 1) {
                                    currentHoldIndex++
                                }
                            }
                            function decreaseHoldTime() {
                                if (currentHoldIndex > 0) {
                                    currentHoldIndex--
                                }
                            }
                            function scaleUpY() {
                                if (graph.yScale < 512) {
                                    graph.yScale *= 2
                                    content.contentY *= 2
                                }
                            }
                            function scaleDownY() {
                                if (graph.yScale > (1/16)) {
                                    graph.yScale /= 2
                                    content.contentY /= 2
                                }
                            }
                            function scaleUpX() {
                                if (graph.xScale < 512) {
                                    graph.xScale *= 2

                                }
                            }
                            function scaleDownX() {
                                if (graph.xScale > (1/16)) {
                                    graph.xScale /= 2
                                }
                            }

                            /// Timer that executes samples removal. Only sample older than holdTime seconds
                            /// will be removed. This timer should be left here.
                            Timer {
                                id: timer
                                interval: 50 // This can be changed to something like 100 or 200.
                                repeat: true
                                running: true
                                onTriggered: {
                                    graph.removeOldSamples();
                                    /// Demo function - can be removed after desired source of data is implemented
                                    // graph.appendSample(graph.newSample(++graph.offset));
                                }
                            }
                            property int offset: 100
                            /// This mouse area handles zooming in and out. Some values can be edited to adjust
                            /// scale range. Remember to keep scale values as powers of 2.
                            MouseArea {
                              anchors.fill: parent
                              onWheel: {

                                console.log("Wheel event, angle delta is:",wheel.angleDelta)
                                /// 512 can be change to 256 to lower the scale range or 1024 to increase
                                /// scaling range
                                if(wheel.angleDelta.y > 0) {
                                    graph.scaleUpY()
                                }

                                /// 1/16 can be change to 1/8 to make the minimal scale larger
                                /// or 1/32 make the minimal scale smaller.
                                else if (wheel.angleDelta.y < 0) {
                                    graph.scaleDownY()
                                }

                                console.log("Scale is:",graph.scale)
                                console.log("Content height is:",content.contentHeight)
                              }
                            }
                            /// Connections prepared for real data source. You need to specify the target
                            /// that emits certain signal and handle that signal.
                            Connections {
                                target: zpathdata // can be changed to any QObject that emits signals
                                /// Signal handler - it will be called after depthChanged signal is emitted
                                onZPathChanged: {
                                   //console.log("Z data is",zpathdata.zpathdata)
                                   // Uncomment this line to populate graph.
                                   graph.appendSample(zpathdata.zpathdata)
                                }
                            }
                        }

                    }
                }
            }
        }
    }


    Rectangle {
        id: dashboard
        anchors {
            left: parent.left; right: parent.right
        }
        height: 50
        color: "#88000000"

        Rectangle {
            color: "transparent"
            anchors.left: buttonSwitch.right
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin: 5
            width: 30
            height: 30
            Image {
                anchors.fill:parent
                source: "qrc:/gps_arrow.png"
                MouseArea {
                    anchors.fill:parent
                    onClicked: stack.currentIndex = !stack.currentIndex
                }
            }
        }



        MouseArea {
            id: buttonSwitch
            anchors {
                left: parent.left; leftMargin: 30
                verticalCenter: parent.verticalCenter
            }
            width: 30; height: 30
            scale: pressed ? 1.1 : 1.0
            cursorShape: Qt.PointingHandCursor

            Image {
                anchors.fill: parent
                source: "qrc:/switch.png"
            }

            onClicked: {
                mShowState = !mShowState
            }
        }

        Row {
            anchors {
                left: buttonSwitch.right; leftMargin: 50
                verticalCenter: parent.verticalCenter
            }

            Text {
                id: lblrpm
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "RPM:"
            }

            Text {
                id: textrpm
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                rightPadding: 10
                color: "white"
                text: toFixed(rpmdata.rpmdata, 2)

                property var rpm:0
            }

            Text {
                id: lblthrottle
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Throttle:"
            }

            Text {
                id:throttletext
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(throttledata.throttledata, 2)+ "N"
            }

            Text {
                id: lblspeed
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Speed:"
            }

            Text {
                id: textspeed
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                rightPadding: 10
                color: "white"
                text: Math.round(speed.speed) + " knots"
            }

            Text {
                id: lblPressureBallast
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Ballas Pressure:"
            }

            Text {
                id: textPressureBallast
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(pressuredata.pressuredata, 2)
            }
            Text {
                id: lbldepth
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Depth:"
            }

            Text {
                id: textdepth
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(depthdata.depthdata, 2)
            }
            Text {
                id: lblBattery
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Battery:"
            }

            Text {
                id: batterytext
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: Math.round(battery.battery) + "%"
            }

            Text {
                id: lblCurrent
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Current:"
            }

            Text {
                id: currenttext
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(currentdata.currentdata, 2) + "A"
            }

            Text {
                id: lblCompass
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Compass:"
            }

            Text {
                id: txtCompass
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: Math.round(yaw.yaw)
            }

            Text {
                id: lblRolldata
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Roll:"
            }

            Text {
                id: txtRolldata
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(roll.roll, 2)
            }

            Text {
                id: lblPitchdata
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12; bold: true }
                rightPadding: 5
                color: "white"
                text: "Pitch:"
            }

            Text {
                id: txtPitchdata
                anchors.verticalCenter: parent.verticalCenter
                font { pixelSize: 12 }
                color: "white"
                rightPadding: 10
                text: toFixed(pitch.pitch, 2)
            }

        }

        Image {
            id: imgBattery
            anchors {
                right: parent.right; rightMargin: 30
                verticalCenter: parent.verticalCenter
            }
            height: 30
            fillMode: Image.PreserveAspectFit
            source: "qrc:/battery.png"

            Rectangle {
                anchors {
                    left: parent.left; leftMargin: 5
                    verticalCenter: parent.verticalCenter
                }
                radius: 2
                height: parent.height - 10
                width: (parent.width - 14) * battery.battery / 100
            }
        }
    }


    function toFixed(number, precision) {
        return number === parseInt(number) ? parseInt(number) : parseFloat(number).toFixed(precision)
    }
}







