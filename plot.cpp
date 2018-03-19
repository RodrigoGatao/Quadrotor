#include "plot.h"
#include"qcustomplot.h"


plot::plot(QCustomPlot *parent)
{
        m_parent = parent;
        m_graph = m_parent->addGraph();
}

void plot::draw_graph(){
    m_graph->setData(mx,my);
}

void plot::clear(){
    mx.clear();
    my.clear();
    m_graph->setData(mx,my);
}
