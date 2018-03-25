#ifndef PLOT_H
#define PLOT_H
#include<QVector>
#include<iostream>

using namespace std;

class QCPGraph;
class QCustomPlot;

class plot
{
private:
    QCPGraph *m_graph;
    QCustomPlot *m_parent;
    QVector<double> mx;
    QVector<double> my;
public:
    plot(QCustomPlot *parent);

    /*****************
     * Miscelanius Functions *
     ****************/
    /**
     * @brief clear The mx and my variables
     */
    void clear();
    /**
     * @brief draw_graph add a new data to the graphic
     */
    void draw_graph();

    /*****************
     *  Set Functions *
     ****************/
    /**
     * @brief set_des_state defines the desire state in a current time
     */
    void set_des_state();
    /**
     * @brief set_state defines the state in a current time
     */
    void set_state();
    /**
     * @brief set_x add a new point value to the mx vector
     * @param x
     */
    void set_x(double x);
    /**
     * @brief set_y add a new point value to the my vector
     * @param y
     */
    void set_y(double y);

};

#endif // PLOT_H
