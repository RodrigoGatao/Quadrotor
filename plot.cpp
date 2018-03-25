#include "plot.h"
#include"qcustomplot.h"

plot::plot(QCustomPlot *parent)
{
        m_parent = parent;
        m_graph = m_parent->addGraph();
}

/*****************
 * Miscelanius Functions *
 ****************/
void plot::clear(){
    mx.clear();
    my.clear();
    m_graph->setData(mx,my);
}

void plot::draw_graph(){
    m_graph->setData(mx,my);
}


/*****************
 * Set Functions *
 ****************/

void plot::set_des_state(){
    QPen pen;
    pen.setColor("Blue");
    pen.setStyle(Qt::DashLine);
    m_graph->setPen(pen);
}

void plot::set_state(){
    QPen pen;
    pen.setColor("Black");
    pen.setStyle(Qt::SolidLine);
    m_graph->setPen(pen);
}

void plot::set_x(double x)
{
    mx << x;
}

void plot::set_y(double y)
{
    my << y;
}
