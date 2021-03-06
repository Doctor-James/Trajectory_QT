#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <vector>
#include <eigen3/Eigen/Core>
#include "armor_msg.hpp"
#include <lcm/lcm-cpp.hpp>
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    struct param
    {
        param(float ang,float spe,float dis)
        {
            d_angle = ang;
            speed = spe;
            distance = dis;
        }
        float d_angle;
        float speed;
        float distance;
    };
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    // lcm
    lcm::LCM *m_lcm;
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const armor_msg *msg)
    {
        m_armor[0] = msg->position[0];
        m_armor[1] = msg->position[1];
        m_armor[2] = msg->position[2];
        qDebug()<<"armor: "<<m_armor[0];
    }
    void get_lcm(lcm::LCM *lcm)
    {
        m_lcm = lcm;
    }
private slots:
    void on_pushButton_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

    void on_pushButton_2_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_horizontalSlider_2_sliderMoved(int position);


    void on_up_valueChanged(double arg1);

    void on_forward_valueChanged(double arg1);

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_speed_set_valueChanged(double arg1);
    void serial_port();
    void armor_lcm();
private:
    Ui::MainWindow *ui;
    QTimer *timer_sp;
    QTimer *timer_armor;
    QString m_mode_string = "pitch";
    int m_set_yaw = 0;
    int m_set_pitch = 0;
    float m_pitch_decimal = 0;
    float m_yaw_decimal = 0;

    float m_set_speed;
    float m_mechanical_up;
    float m_mechanical_forward;
    int m_hit_num = 0;
    int m_miss_num = 0;
    bool m_is_shoot = false;

    Eigen::Vector3f m_armor;
    std::vector<param> m_yaw_param;
    std::vector<param> m_pitch_param;
    float m_get_yaw = -10;
    float m_get_pitch = -10;
    // yaw ??????????????????????????????
    Eigen::Vector2f m_yaw_ceo;
    // pitch ???????????????????????????
    Eigen::Vector3f m_pitch_ceo;

    void disable_ui();
    // ????????????,??????????????????????????????
    Eigen::Vector2f fit_curve_1(std::vector<param> params);
    Eigen::Vector3f fit_curve_2(std::vector<param> params);
    // ???????????????????????????
    float adjust_angle1(Eigen::Vector2f ceo,float value);
    float adjust_angle2(Eigen::Vector3f ceo,float value);
    // ?????????????????????????????????pitch???yaw
    Eigen::Vector2f ballistic_equation(Eigen::Vector3f armor);
    float forward_ballistic_equation(float angle,float x);
    void read_param();


};
#endif // MAINWINDOW_H
