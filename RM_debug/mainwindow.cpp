#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fstream>

double armor[3];
static void armor_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const armor_msg * msg, void * user)
{
    armor[0] = msg->position[0];
    armor[1] = msg->position[1];
    armor[2] = msg->position[2];
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_pitch_ceo << 0,0,0;
    m_yaw_ceo << 0,0;
    disable_ui();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disable_ui()
{
    ui->angle_set->setEnabled(false);
    ui->up->setEnabled(false);
    ui->forward->setEnabled(false);
    ui->hits->setEnabled(false);
    ui->misses->setEnabled(false);
    ui->pct->setEnabled(false);
    ui->pitch->setEnabled(false);
    ui->yaw->setEnabled(false);
    ui->state->setEnabled(false);
    ui->speed_set->setEnabled(false);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_5->setEnabled(false);
    ui->pushButton_6->setEnabled(false);
    ui->pushButton_7->setEnabled(false);
    ui->pushButton_8->setEnabled(false);
    ui->horizontalSlider->setEnabled(false);
    ui->horizontalSlider_2->setEnabled(false);
    ui->ceo1->setEnabled(false);
    ui->ceo2->setEnabled(false);
    ui->ceo3->setEnabled(false);
}

// 初始化
void MainWindow::on_pushButton_clicked()
{
    disable_ui();
    // 初始云台角度
    m_set_yaw = 0;
    m_set_pitch = 0;
    // ui 控件启动
    ui->pushButton_2->setEnabled(true);
    ui->speed_set->setEnabled(true);
    ui->pitch->setEnabled(true);
    ui->yaw->setEnabled(true);
    ui->state->setEnabled(true);
    ui->ceo1->setValue(0);
    ui->ceo2->setValue(0);
    ui->ceo3->setValue(0);
    if(m_mode_string == "yaw")
    {
        ui->horizontalSlider->setEnabled(true);
        ui->horizontalSlider_2->setEnabled(true);
        ui->angle_set->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->pushButton_4->setEnabled(true);
        ui->pushButton_5->setEnabled(true);
        ui->angle_set->setEnabled(true);
        ui->up->setEnabled(true);
        ui->forward->setEnabled(true);
        ui->pushButton_6->setEnabled(true);
        ui->ceo1->setEnabled(true);
        ui->ceo2->setEnabled(true);
        ui->ceo3->setEnabled(true);
        m_yaw_ceo<<0,0;
    }
    else if(m_mode_string == "pitch")
    {
        ui->horizontalSlider->setEnabled(true);
        ui->horizontalSlider_2->setEnabled(true);
        ui->angle_set->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->pushButton_4->setEnabled(true);
        ui->pushButton_5->setEnabled(true);
        ui->angle_set->setEnabled(true);
        ui->up->setEnabled(true);
        ui->forward->setEnabled(true);
        ui->pushButton_6->setEnabled(true);
        ui->ceo1->setEnabled(true);
        ui->ceo2->setEnabled(true);
        ui->ceo3->setEnabled(true);
        m_pitch_ceo<<0,0,0;
    }
    else
    {// both
        ui->hits->setEnabled(true);
        ui->misses->setEnabled(true);
        ui->pct->setEnabled(true);
        ui->pushButton_7->setEnabled(true);
        ui->pushButton_8->setEnabled(true);
        read_param();
    }
    // 定时调用函数
    timer_sp = new QTimer(this);
    connect(timer_sp,SIGNAL(timeout()),this,SLOT(serial_port()));
    timer_sp->start(5);//5ms 调用一次
    timer_armor = new QTimer(this);
    connect(timer_armor,SIGNAL(timeout()),this,SLOT(armor_lcm()));
    timer_armor->start(5);//5ms 调用一次
    //@TODO
    //1、添加初始化串口代码
    //2、添加初始化相机代码
}

// pitch yaw both 选择
void MainWindow::on_comboBox_currentTextChanged(const QString &arg1)
{
    m_mode_string = arg1;
}

// 发射子弹命令
void MainWindow::on_pushButton_2_clicked()
{
    m_is_shoot = true;
    qDebug()<<"shoot !!";
}

// 粗调整角度 单位 1 度
void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    if(m_mode_string == "yaw")
    {
        m_set_yaw = position;
        ui->angle_set->setValue(m_set_yaw + m_yaw_decimal);
        qDebug()<<"set yaw angle : "<<m_set_yaw + m_yaw_decimal;
    }
    else if(m_mode_string == "pitch")
    {
        m_set_pitch = position;
        ui->angle_set->setValue(m_set_pitch + m_pitch_decimal);
        qDebug()<<"set pitch angle : "<<m_set_pitch + m_pitch_decimal;
    }
}

// 精调整角度 单位 0.01 度
void MainWindow::on_horizontalSlider_2_sliderMoved(int position)
{
    if(m_mode_string == "yaw")
    {
        m_yaw_decimal = position * 0.01f;
        ui->angle_set->setValue(m_set_yaw + m_yaw_decimal);
        qDebug()<<"set yaw angle : "<<m_set_yaw + m_yaw_decimal;
    }
    else if(m_mode_string == "pitch")
    {
        m_pitch_decimal = position * 0.01f;
        ui->angle_set->setValue(m_set_pitch + m_pitch_decimal);
        qDebug()<<"set pitch angle : "<<m_set_pitch + m_pitch_decimal;
    }
}


// 机械安装 向上
void MainWindow::on_up_valueChanged(double arg1)
{
    m_mechanical_up = arg1;
    qDebug()<<"set mechanical up : "<<m_mechanical_up;
}

// 机械安装 向前
void MainWindow::on_forward_valueChanged(double arg1)
{
    m_mechanical_forward = arg1;
    qDebug()<<"set mechanical forward : "<<m_mechanical_forward;
}

// hit
void MainWindow::on_pushButton_7_clicked()
{
    m_hit_num ++;
    ui->hits->setValue(m_hit_num);
    ui->pct->setValue((float)m_hit_num/(m_hit_num + m_miss_num));
}

// miss
void MainWindow::on_pushButton_8_clicked()
{
    m_miss_num ++;
    ui->misses->setValue(m_miss_num);
    ui->pct->setValue((float)m_hit_num/(m_hit_num + m_miss_num));
}

// 记录
void MainWindow::on_pushButton_3_clicked()
{
    float distance = m_armor.norm();
    if(m_mode_string == "yaw")
    {
        m_yaw_param.emplace_back(param(m_set_yaw,m_set_speed,distance));
    }
    else if(m_mode_string == "pitch")
    {
        m_pitch_param.emplace_back(param(m_set_pitch,m_set_speed,distance));
    }
}

// 标定参数
void MainWindow::on_pushButton_4_clicked()
{
    if(m_mode_string == "yaw")
    {// 拟合一次曲线
        m_yaw_ceo = fit_curve_1(m_yaw_param);
    }
    else if(m_mode_string == "pitch")
    {// 拟合二次曲线
        m_pitch_ceo = fit_curve_2(m_pitch_param);
    }
}

// 保存参数
void MainWindow::on_pushButton_5_clicked()
{
    std::ofstream file("../RM_debug/param.txt",std::ios::out);
    if (file.is_open())
    {
        file << m_yaw_ceo[0]<<" "<<m_yaw_ceo[1]<<"\n";
        file << m_pitch_ceo[0]<<" "<<m_pitch_ceo[1]<<" "<<m_pitch_ceo[2]<<"\n";
        file << m_mechanical_up<<" "<<m_mechanical_forward<<"\n";
        file.close();
    }
    else
    {
        qDebug()<<"can not open the file!";
    }
}
// 读取参数
void MainWindow::read_param()
{
    std::string str;
    std::ifstream file("../RM_debug/param.txt",std::ios::in);
    file >> str;
    m_yaw_ceo[0] = std::stof(str);
    file >> str;
    m_yaw_ceo[1] = std::stof(str);
    file >> str;
    m_pitch_ceo[0] = std::stof(str);
    file >> str;
    m_pitch_ceo[1] = std::stof(str);
    file >> str;
    m_pitch_ceo[2] = std::stof(str);
    file >> str;
    m_mechanical_up = std::stof(str);
    file >> str;
    m_mechanical_forward = std::stof(str);
    ui->up->setValue(m_mechanical_up);
    ui->forward->setValue(m_mechanical_forward);
}

// 清除参数
void MainWindow::on_pushButton_6_clicked()
{
    m_pitch_ceo << 0,0;
    m_yaw_ceo << 0,0,0;
    m_pitch_param.clear();
    m_yaw_param.clear();
}

// 设定速度
void MainWindow::on_speed_set_valueChanged(double arg1)
{
    m_set_speed = arg1;
    qDebug()<<"set speed : "<<m_set_speed;
}

// 定时器调用
void MainWindow::armor_lcm()
{
    m_lcm->handle();
}

// 定时器调用
void MainWindow::serial_port()
{
    bool state = false;// 是否读到数据

    float send_yaw = 0;
    float send_pitch = 0;
    //@TODO 弹道方称计算
    auto angle = ballistic_equation(m_armor);
    //@TODO 弹道方程标定校准
    if(m_mode_string == "yaw")
    {
        send_yaw = angle[1] + m_set_yaw;
        send_pitch = angle[0];
    }
    else if(m_mode_string == "pitch")
    {
        send_yaw = angle[1];
        send_pitch = angle[0] + m_set_pitch;
    }
    else
    {// 测试模式，用标定好的参数计算得到 send_yaw send_pitch
        float distance = m_armor.norm();
        send_yaw = angle[1] + adjust_angle1(m_yaw_ceo, distance);
        send_pitch = angle[0] + adjust_angle2(m_pitch_ceo, distance);
    }
    //@TODO 串口发送当前的设定角度

    //@TODO 更新当前数据

    m_is_shoot = false;// 发送数据后置为空
    // 更新ui界面
    ui->yaw->setValue(m_get_yaw);
    ui->pitch->setValue(m_get_pitch);
    ui->state->setChecked(state);
}

// 一次曲线拟合
Eigen::Vector2f MainWindow::fit_curve_1(std::vector<param> params)
{
    float A = 0;   //x的平方求和
    float B = 0;   //x求和
    float C = 0;   //xy求和
    float D = 0;   //y求和
    float E = 0;    //样本个数
    for (int i = 0; i <= params.size(); i++)
    {
        A = A + pow(params[i].distance, 2);

        B = B + params[i].distance;

        C = C + params[i].distance * params[i].d_angle;

        D = D + params[i].d_angle;
        E = params.size();
    }
    float a, b = 0;
    if (E * A - B * B == 0)
    {
        a = 1;
        b = 0;
    }
    else
    {
        a = (E * C - B * D) / (E * A - B * B);
        b = (A * D - B * C) / (E * A - B * B);
    }
    Eigen::Vector2f result;
    result << a,b;
    return result;
}

// 二次曲线拟合
Eigen::Vector3f MainWindow::fit_curve_2(std::vector<param> params)
{
    //-----------------公式---------------------------
    //n A1     + sumX A2   + sumXX A3 = sumY
    //sumX A1  + sumXX A2  + sumXXX A3 = sumXY
    //sumXX A1 + sumXXX A2 + sumXXXX A3 = sumXXY
    //--------------------------------------------目的：求系数A1 A2 A3
    long long int sumX = 0, sumY = 0, sumXX = 0, sumXXX = 0, sumXXXX = 0, sumXY = 0, sumXXY = 0;//建立方程组
    for (int i = 0; i < params.size(); i = i ++)
    {
        sumX = sumX + params[i].distance;
        sumY = sumY + params[i].d_angle;
        sumXX = sumXX + params[i].distance*params[i].distance;
        sumXXX = sumXXX + params[i].distance*params[i].distance*params[i].distance;
        sumXXXX = sumXXXX + params[i].distance*params[i].distance*params[i].distance*params[i].distance;
        sumXY = sumXY + params[i].distance*params[i].d_angle;
        sumXXY = sumXXY + params[i].distance*params[i].d_angle*params[i].distance;
    }
    double a11 = params.size();      double  a12 = sumX;   double  a13 = sumXX;   double  b1 = sumY;      //构造矩阵   求解方程系数   a11 第一行第一列
    double  a21 = sumX;  double  a22 = sumXX;  double  a23 = sumXXX;  double  b2 = sumXY;
    double  a31 = sumXX; double  a32 = sumXXX; double  a33 = sumXXXX; double  b3 = sumXXY;

    //double a11 = 2;      double  a12 = 4;   double  a13 = -2;   double  b1 = 2;      //构造矩阵   求解方程系数   a11 第一行第一列
    //double  a21 = 1;  double  a22 = -3;  double  a23 = -3;  double  b2 = -1;
    //double  a31 = 4; double  a32 = 2; double  a33 = 2; double  b3 = 3;

    //R2-(a21/a11)*R1     ------------------------------------------------------------ - 进行消元    高斯消元法
    double a21x, a22x, a23x, b2x;
    a21x = a21 - a21 / a11*a11; a22x = a22 - a21 / a11*a12; a23x = a23 - a21 / a11*a13; b2x = b2 - a21 / a11*b1;
    //R3-(a31/a11)*R1
    double a31x, a32x, a33x, b3x;
    a31x = a31 - a31 / a11*a11; a32x = a32 - a31 / a11*a12; a33x = a33 - a31 / a11*a13; b3x = b3 - a31 / a11*b1;

    //R3-(a32/a22)*R2
    double a32xx, a33xx, b3xx;
    a32xx = a32x - a32x / a22x*a22x; a33xx = a33x - a32x / a22x*a23x; b3xx = b3x - a32x / a22x*b2x;

    //------------------------------------------------------------求得系数矩阵未知数
    double A3 = b3xx / a33xx;
    double A2 = (b2x - a23x*A3) / a22x;
    double A1 = (b1 - A2*a12 - A3*a13) / a11;

    Eigen::Vector3f result;
    result <<A1,
             A2,
             A3;
    return result;
}

// 通过参数获得修正量
float MainWindow::adjust_angle1(Eigen::Vector2f ceo,float value)
{
    return ceo[0] + ceo[1] * value;
}
float MainWindow::adjust_angle2(Eigen::Vector3f ceo,float value)
{
    return ceo[0] + ceo[1] * value + ceo[2] * value * value;
}

// 根据物理方程来计算设定pitch和yaw
Eigen::Vector2f MainWindow::ballistic_equation(Eigen::Vector3f armor)
{
    Eigen::Vector2f result;

    // 先计算yaw轴角度
    float yaw = atan(armor[1] / armor[0]);
    // armor 的位置进行了一定的旋转
    armor << sqrt(armor[0] * armor[0] + armor[1] * armor[1]), 0, armor[2];
    // 计算pitch轴的初始角度
    float pitch = atan(armor[2] / armor[0]);
    // 迭代计算pitch轴角度
    for(int i = 0;i < 10;i ++)
    {// 多次迭代计算得到目标位置
        // 前向弹道方程
        auto target = forward_ballistic_equation(pitch,armor[0]);
        auto err = armor[2] - target;
        if(err < 0.001)
        {
            break;
        }
        // 计算导数
//        float cos2 = ;
//        float cos3 = ;

        float J = 0;
        float d_theta = - err / J;
        // 更新设定角度
        pitch += d_theta;
    }
    result << 0,0;
    return result;
}

// 高中物理公式
float MainWindow::forward_ballistic_equation(float angle,float x)
{
    float time = x / (m_set_speed * cos(angle * 3.14159 / 180));
    return m_set_speed * sin(angle * 3.14159 / 180) * time - 0.5 * 9.8 * time * time;
}


