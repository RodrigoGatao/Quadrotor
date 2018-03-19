#ifndef PLOT_H
#define PLOT_H
#include<QVector>
#include<iostream>

using namespace std;

class QCPGraph;
class QCustomPlot;

class plot
{
public:
    plot(QCustomPlot *parent);
    void draw_graph();
    void set_x(double x){
        mx << x;
    }
    void set_y(double y){
        my << y;
    }
    void clear();
private:
    QCPGraph *m_graph;
    QVector<double> mx;
    QVector<double> my;
    QCustomPlot *m_parent;
};

#endif // PLOT_H
