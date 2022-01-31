#ifndef GRAPH_H
#define GRAPH_H

#include <QQuickItem>

#include "linenode.h"


/// Graph control for plotting real time data with time based x axis.
/// This control is registered to QML Engine and can be created as separate control.
/// To create a control user must import in .qml file 'import graph 1.0'.
/// To add new value to existing samples - appendSample() method must be called.
/// This method can be called directly from .qml file.  (for example when new value comes from
/// ROS topic). User can configure the graph by using yMin and yMax properties.
/// These properties configure the range of the whole graph. Any value outside of this range will not be visible
/// on graph (it would be rendered outside the control). User can also specify the scale of the graph to
/// implement zoom in/out functionality. As graph works with real time data it has flushing mechanism.
/// The graph removes old values after some time determined by holdTime property. Default value for holding time
/// is 60 seconds and can be changed using a property. The flushing mechanism is not enabled by default and must be called
/// explicitly using for example a timer.
class Graph : public QQuickItem
{
    Q_OBJECT
    /// Property that holds minimal y value that will be displayed graph.
    Q_PROPERTY(float yMin READ getYMin WRITE setYMin NOTIFY yMinChanged)
    /// Property that holds maximum y value that will be displayed on graph.
    Q_PROPERTY(float yMax READ getYMax WRITE setYMax NOTIFY yMaxChanged)
    /// Property that holds scale of the graph.
    Q_PROPERTY(float xScale READ getXScale WRITE setXScale NOTIFY scaleXChanged)
    Q_PROPERTY(float yScale READ getYScale WRITE setYScale NOTIFY scaleYChanged)

    /// Property that holds holdTime after which old values will be removed from the graph.
    Q_PROPERTY(int holdTime READ getHoldTime WRITE setHoldTime NOTIFY holdTimeChanged)

public:
    Graph();
    float getYMin() const { return m_plotConfig.yMin; }
    float getYMax() const { return m_plotConfig.yMax; }
    float getXScale() const {return m_plotConfig.xScale;}
    float getYScale() const {return m_plotConfig.yScale;}


    int getHoldTime() const { return m_holdTime; }

    void setYMin(float a_yMin);
    void setYMax(float a_yMax);
    void setXScale(float a_xScale);
    void setYScale(float a_yScale);
    void setHoldTime(int a_holdTime);
protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);
    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry);

public slots:
    void appendSample(qreal value);
    void removeFirstSample();
    void removeOldSamples();

signals:
  void yMinChanged();
  void yMaxChanged();
  void scaleXChanged();
  void scaleYChanged();
  void holdTimeChanged();

private:
    QList<Data> m_samples;
    PlotConfig m_plotConfig{};
    bool m_samplesChanged;
    bool m_geometryChanged;
    bool m_plotConfigChanged{};
    int m_holdTime{60};
};

#endif // GRAPH_H
