/****************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "linenode.h"

#include <QtGui/QColor>

#include <QtQuick/QSGSimpleMaterial>

struct LineMaterial
{
    QColor color;
    float spread;
    float size;
};

class LineShader : public QSGSimpleMaterialShader<LineMaterial>
{
    QSG_DECLARE_SIMPLE_SHADER(LineShader, LineMaterial)

public:
    LineShader() {
        setShaderSourceFile(QOpenGLShader::Vertex, ":/scenegraph/graph/shaders/line.vsh");
        setShaderSourceFile(QOpenGLShader::Fragment, ":/scenegraph/graph/shaders/line.fsh");
    }

    QList<QByteArray> attributes() const override {  return QList<QByteArray>() << "pos" << "t"; }

    void updateState(const LineMaterial *m, const LineMaterial *) override {
        program()->setUniformValue(id_color, m->color);
        program()->setUniformValue(id_spread, m->spread);
        program()->setUniformValue(id_size, m->size);
    }

    void resolveUniforms() override {
        id_spread = program()->uniformLocation("spread");
        id_size = program()->uniformLocation("size");
        id_color = program()->uniformLocation("color");
    }

private:
    int id_color = -1;
    int id_spread = -1;
    int id_size = -1;
};

struct LineVertex {
    float x;
    float y;
    float t;
    inline void set(float xx, float yy, float tt) { x = xx; y = yy; t = tt; }
};

static const QSGGeometry::AttributeSet &attributes()
{
    static QSGGeometry::Attribute attr[] = {
        QSGGeometry::Attribute::create(0, 2, GL_FLOAT, true),
        QSGGeometry::Attribute::create(1, 1, GL_FLOAT)
    };
    static QSGGeometry::AttributeSet set = { 2, 3 * sizeof(float), attr };
    return set;
}

LineNode::LineNode(float size, float spread, const QColor &color)
    : m_geometry(attributes(), 0)
{
    setGeometry(&m_geometry);
    m_geometry.setDrawingMode(GL_TRIANGLE_STRIP);

    QSGSimpleMaterial<LineMaterial> *m = LineShader::createMaterial();
    m->state()->color = color;
    m->state()->size = size;
    m->state()->spread = spread;
    m->setFlag(QSGMaterial::Blending);
    setMaterial(m);
    setFlag(OwnsMaterial);
}

/*
 * Assumes that samples have values in the range of 0 to 1 and scales them to
 * the height of bounds. The samples are stretched out horizontally along the
 * width of the bounds.
 *
 * The position of each pair of points is identical, but we use the third value
 * "t" to shift the point up or down and to add antialiasing.
 */
void LineNode::updateGeometry(const QRectF &bounds, const QList<Data> &samples)
{
    m_geometry.allocate(samples.size() * 2);

    float x = bounds.x();
    float y = bounds.y();
    float w = bounds.width();
    float h = bounds.height();
    if(m_plotConfig.yScale < 1){
        h *= m_plotConfig.yScale;
    }
    if(m_plotConfig.xScale < 1) {
        w *= m_plotConfig.xScale;
    }

    float dx = w / (samples.size() - 1);

    const auto calcY = [=](float a_y) {
      return ((a_y - m_plotConfig.yMin) * (h - 0)) /
                 (m_plotConfig.yMax - m_plotConfig.yMin) +
             0;
    };
    LineVertex *v = (LineVertex *) m_geometry.vertexData();
    float timeDelta{-1.0f};
      if (samples.size() > 1) {
        // qDebug() << "First time is: " << samples.begin()->timestamp.toString();
        // qDebug() << "Last time is: " << samples.last().timestamp.toString();
        timeDelta = samples.first().timestamp.msecsTo(samples.last().timestamp);
        // qDebug() << "Time delta is: " << timeDelta;
      }
      for (int i = 0; i < samples.size(); ++i) {
        float xPosition = 0.0f;
        if (timeDelta > 0.0) {
          float currentTimePosition =
              samples.at(0).timestamp.msecsTo(samples.at(i).timestamp);
          xPosition = ((currentTimePosition - 0) * (w - 0)) / (timeDelta - 0.0) + 0;
        }
        v[i * 2 + 0].set(xPosition,
                         y + bounds.height() - calcY(samples.at(i).value), 0);
        v[i * 2 + 1].set(xPosition,
                         y + bounds.height() - calcY(samples.at(i).value), 1);
      }

    markDirty(QSGNode::DirtyGeometry);
}

void LineNode::setPlotConfig(const PlotConfig &a_plotConfig) {
  m_plotConfig = a_plotConfig;
  markDirty(QSGNode::DirtyGeometry);
}

