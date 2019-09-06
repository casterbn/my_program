using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Collections;

namespace aceinna_com
{

    public partial class Form1 : Form
    {
        //声明变量
        FileStream fs;

        SerialPort sp = new SerialPort();
        bool isSetProperty = false;//串口属性设置标志位
        private enum PortState//声明接口显示状态，枚举型
        {
            打开,
            关闭
        }
        string path = AppDomain.CurrentDomain.BaseDirectory + "confing.ini";//声明配置文件路径
        string tbSendDataStr = "";//发送窗口字符串存储
        string tbSendData16 = "";//发送窗口16进制存储
        List<byte> receivedDatas = new List<byte>();//接收数据泛型数组
        int num = 0;
        //接收串口数据
        private void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (!isSetProperty)
                {
                    return;
                }
                if (sp.BytesToRead == null)
                {
                    return;
                }
                byte[] ReceivedData = new byte[sp.BytesToRead];//创建接收字节数组
                List<byte> write_to_file = new List<byte>();
                sp.Read(ReceivedData, 0, ReceivedData.Length);//读取所接收到的数据
                string str = BitConverter.ToString(ReceivedData);
                write_to_file.AddRange(ReceivedData);
                string str_to_file;
                if (check_display.Checked == true)
                {
                    receivedDatas.AddRange(ReceivedData);
                }
                tbReceivedCount.Text = (Convert.ToInt32(tbReceivedCount.Text) + ReceivedData.Length).ToString();

                if (cb16Display.Checked)
                {
                    if (check_display.Checked == true)
                    {
                        //tbReceivedData.Text = Methods.ByteTo16Str(receivedDatas.ToArray());
                        tbReceivedData.Text += Methods.ByteTo16Str(ReceivedData.ToArray());
                        //tbReceivedData.Clear();
                    }
                    str_to_file = Methods.ByteTo16Str(write_to_file.ToArray());
                }
                else
                {
                    if (check_display.Checked == true)
                    {
                        tbReceivedData.Text = Encoding.Default.GetString(receivedDatas.ToArray());
                        tbReceivedData.Clear();
                    }
                    str_to_file = Encoding.Default.GetString(write_to_file.ToArray());
                }


#if true
                string fName = textBox1.Text;
                FileStream fs_test = null;
                if (fs_test == null)
                {
                    //fs_test = File.Open(fName, FileMode.Append);
                }
                if ((save_file_check.Checked == true))
                {
                    //BinaryWriter bw = new BinaryWriter(new FileStream("0708", FileMode.Create));
                    //bw.Write("123");
                    //bw.Write(ReceivedData, 0, ReceivedData.Length);
                    if (bin_check.Checked == true)
                    {
                        num += 1;
                        tbSendCount.Text = num.ToString();
                        //FileStream f_bin = new FileStream(fName, FileMode.OpenOrCreate, FileAccess.ReadWrite);
                        FileStream f_bin = File.Open(fName, FileMode.Append);
                        //实例化FileStream对象，saveFileDialog1.FileName为用户输入的文件名
                        BinaryWriter w = new BinaryWriter(f_bin);
                        //实例化BinaryWriter对象
                        //w.Write("123");
                        w.Write(ReceivedData);
                        w.Close();//关闭二进制写入流
                        f_bin.Close();//关闭文件流
                    }
                    else
                    {
                        using (FileStream fs = File.Open(fName, FileMode.Append))
                    
                        if (str_to_file != null)
                        {
                            if (bin_check.Checked == true)
                            {
                                if (check_display.Checked == true)
                                {
                                    //tbReceivedData.Text = Methods.ByteTo16Str(receivedDatas.ToArray());
                                    //tbReceivedData.Text += Methods.ByteTo16Str(ReceivedData.ToArray());
                                    //tbReceivedData.Clear();
                                }
                                byte[] txt = ReceivedData;
                                if ((txt != null) && (ReceivedData != null))
                                {
                                    //fs.Write(txt, 0, txt.Length); //valid
                                    fs.Write(ReceivedData.ToArray(), 0, ReceivedData.Length); //valid
                                }
                            }
                            else
                            {
                                str_to_file = str_to_file.Replace("D3 00", "\r\nD3 00");
                                byte[] txt = System.Text.Encoding.Default.GetBytes(str_to_file);
                                if (txt != null)
                                {
                                    fs.Write(txt, 0, txt.Length); //valid
                                }
                            }

                        }

                    }
                }
#endif

                //sp.DiscardInBuffer();//丢弃接收缓冲区数据
            }
            catch
            {
                return;
            }
        }

        //发送串口数据
        int send_count = 0;
        private void DataSend()
        {
            try
            {
                if (cb16Send.Checked)
                {

                    byte[] hexBytes = new byte[250];
                    tbSendData16 = tbSendData.Text.Trim();
                    hexBytes =   Methods._16strToHex(tbSendData16);
                    byte loop = 0;
                    int cks = 0 ;
                    send_count += hexBytes.Length;
                    tbSendCount.Text = send_count.ToString();//hexBytes.Length.ToString();
                    sp.Write(hexBytes, 0, hexBytes.Length);
#if false
                    //for (loop = 5; loop < hexBytes.Length; loop++)
                    tbSendCount.Text = hexBytes.Length.ToString();
     
                    if ((hexBytes[4] > 0)&&(hexBytes[4] < 15))
                    {
                        for (loop = 3; loop < hexBytes.Length; loop++)
                        {
                            //cks = cks ^ (BitConverter.ToInt16(hexBytes, loop));
                            cks = cks ^ hexBytes[loop];
                            cks_text.Text = cks.ToString("X");
                        }
                    }
                    //hexBytes.SetValue(cks,hexBytes.Length);
                    //
                    Array.Resize(ref hexBytes, hexBytes.Length + 1);
                    hexBytes[hexBytes.Length - 1] = (byte)(cks);
                  //  cks_text.Text = hexBytes[hexBytes.Length - 1].ToString("X");
                    tbSendCount.Text = hexBytes.Length.ToString();
                    sp.Write(hexBytes, 0, hexBytes.Length);
                    //hexBytes[0] = (byte)(cks);
                    //sp.Write(hexBytes, 0, 1);
                    //tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text)).ToString();
#endif
                }
                else
                {
                    sp.WriteLine(tbSendDataStr);
                    tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text) + tbSendDataStr.Length).ToString();
                }

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message.ToString());
                return;
            }
        }

        //设置串口属性
        private void SetPortProperty()
        {
            //sp.PortName = "COM1";//设置串口名
            sp.PortName = cbxCOMPort.Text.Trim();//设置波特率
            sp.BaudRate = Convert.ToInt32(cbxBaudRate.Text.Trim());//设置波特率
            switch (cbxStopBits.Text.Trim())//设置停止位
            {
                case "1": sp.StopBits = StopBits.One; break;
                case "1.5": sp.StopBits = StopBits.OnePointFive; break;
                case "2": sp.StopBits = StopBits.Two; break;
                default: sp.StopBits = StopBits.None; break;
            }
            sp.DataBits = Convert.ToInt32(cbxDataBits.Text.Trim());//设置数据位
            switch (cbxParity.Text.Trim())//设置奇偶校验位 
            {
                case "无": sp.Parity = Parity.None; break;
                case "奇校验": sp.Parity = Parity.Odd; break;
                case "偶校验": sp.Parity = Parity.Even; break;
                default: sp.Parity = Parity.None; break;
            }
            sp.ReadTimeout = 5000;//设置超时时间为5s
            Control.CheckForIllegalCrossThreadCalls = false;//这个类中我们不检查跨线程的调用是否合法(因为.net 2.0以后加强了安全机制,，不允许在winform中直接跨线程访问控件的属性)
            //定义DataReceived事件的委托，当串口收到数据后出发事件
            sp.DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);
        }

        //设置端口显示状态
        private void DisplayPortState(PortState portState)
        {
            toolStripStatusLabel1.Text = cbxCOMPort.Text + "端口 处于" + portState + "状态 " + cbxBaudRate.Text + " " + cbxDataBits.Text + " " + cbxStopBits.Text + " " + cbxParity.Text;
        }

        //重新打开串口
        private void AgainOpenPort()
        {
            if (sp.IsOpen)
            {
                try
                {
                    sp.Close();
                    SetPortProperty();
                    isSetProperty = true;
                    sp.Open();
                }
                catch (Exception)
                {
                    isSetProperty = false;
                    btnOpenCom.Text = "打开串口";
                    DisplayPortState(PortState.关闭);
                    MessageBox.Show("串口无效或已被占用!", "错误提示");
                    return;
                }

                DisplayPortState(PortState.打开);
            }
            else
            {
                DisplayPortState(PortState.关闭);
            }
        }
        private void tbSendData_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                DataSend();

            }
        }
        public Form1()
        {
            InitializeComponent();
            this.tbSendData.KeyDown += new KeyEventHandler(tbSendData_KeyDown);
        }

        private void for_open()
        {
            int i = 0;
            for (i = 0; i < 25; i++)
            {
                cbxCOMPort.SelectedIndex = i;
                    SetPortProperty();
                try
                {
                    sp.Open();
                    btnOpenCom.Text = "关闭串口";
                    DisplayPortState(PortState.打开);
                }
                catch (Exception)
                {
                    //串口打开失败后，串口属性设置标志位设为false
                   // isSetProperty = false;
                    continue;
                }
            }
            MessageBox.Show("串口无效或已被占用!", "错误提示");
        }

        //软件启动时加载事件
        
        private void Form1_Load(object sender, EventArgs e)
        {
           
            #region 加载配置文件
            Hashtable ht = new Hashtable();
            if (File.Exists(path))
            {
                try
                {
                    string myline = "";
                    string[] str = new string[2];
                    using (StreamReader sr = new StreamReader(path))
                    {
                        myline = sr.ReadLine();
                        while (myline != null)
                        {
                            str = myline.Split('=');    //分割成两部分
                            ht.Add(str[0], str[1]);
                            myline = sr.ReadLine();
                        }
                    }
                }
                catch(Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                }
            }
            #endregion
            
            #region 设置窗口为固定大小且不可最大化
            this.MaximumSize = this.Size;
            this.MinimumSize = this.Size;
            this.MaximizeBox = false;
            #endregion

            #region 列出常用的波特率
            cbxBaudRate.Items.Add("1200");
            cbxBaudRate.Items.Add("2400");
            cbxBaudRate.Items.Add("4800");
            cbxBaudRate.Items.Add("9600");
            cbxBaudRate.Items.Add("19200");
            cbxBaudRate.Items.Add("38400");
            cbxBaudRate.Items.Add("43000");
            cbxBaudRate.Items.Add("56000");
            cbxBaudRate.Items.Add("57600");
            cbxBaudRate.Items.Add("115200");
            cbxBaudRate.Items.Add("460800");
            cbxBaudRate.Items.Add("921600");
            if (ht.ContainsKey("cbxBaudRate"))
                cbxBaudRate.SelectedIndex = cbxBaudRate.Items.IndexOf(ht["cbxBaudRate"].ToString());
            else
                cbxBaudRate.SelectedIndex = 3;
            cbxBaudRate.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion

            #region 列出停止位
            cbxCOMPort.Items.Add("COM1");
            cbxCOMPort.Items.Add("COM2");
            cbxCOMPort.Items.Add("COM3");
            cbxCOMPort.Items.Add("COM4");
            cbxCOMPort.Items.Add("COM5");
            cbxCOMPort.Items.Add("COM6");
            cbxCOMPort.Items.Add("COM7");
            cbxCOMPort.Items.Add("COM8");
            cbxCOMPort.Items.Add("COM9");
            cbxCOMPort.Items.Add("COM10");
            cbxCOMPort.Items.Add("COM11");
            cbxCOMPort.Items.Add("COM12");
            cbxCOMPort.Items.Add("COM13");
            cbxCOMPort.Items.Add("COM14");
            cbxCOMPort.Items.Add("COM15");
            cbxCOMPort.Items.Add("COM16");
            cbxCOMPort.Items.Add("COM17");
            cbxCOMPort.Items.Add("COM18");
            cbxCOMPort.Items.Add("COM19");
            cbxCOMPort.Items.Add("COM20");
            cbxCOMPort.Items.Add("COM21");
            cbxCOMPort.Items.Add("COM22");
            cbxCOMPort.Items.Add("COM23");
            cbxCOMPort.Items.Add("COM24");
            cbxCOMPort.Items.Add("COM25");
            cbxCOMPort.Items.Add("COM26");
            cbxCOMPort.Items.Add("COM27");
            cbxCOMPort.Items.Add("COM28");
            cbxCOMPort.Items.Add("COM29");
            cbxCOMPort.Items.Add("COM30");
            cbxCOMPort.Items.Add("COM31");
            cbxCOMPort.Items.Add("COM32");
            cbxCOMPort.Items.Add("COM33");
            cbxCOMPort.Items.Add("COM34");
            cbxCOMPort.Items.Add("COM35");
            cbxCOMPort.Items.Add("COM37");
            cbxCOMPort.Items.Add("COM38");
            cbxCOMPort.Items.Add("COM39");
            cbxCOMPort.Items.Add("COM40");
            cbxCOMPort.Items.Add("COM41");
            cbxCOMPort.Items.Add("COM42");
            cbxCOMPort.Items.Add("COM43");
            cbxCOMPort.Items.Add("COM44");
            cbxCOMPort.Items.Add("COM45");
            cbxCOMPort.Items.Add("COM46");
            cbxCOMPort.Items.Add("COM47");
            if (ht.ContainsKey("cbxCOMPort"))
                cbxCOMPort.SelectedIndex = cbxCOMPort.Items.IndexOf(ht["cbxCOMPort"].ToString());
            else
                cbxCOMPort.SelectedIndex = 1;
            cbxCOMPort.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion


            #region 列出停止位
            cbxStopBits.Items.Add("1");
            cbxStopBits.Items.Add("1.5");
            cbxStopBits.Items.Add("2");
            if (ht.ContainsKey("cbxStopBits"))
                cbxStopBits.SelectedIndex = cbxStopBits.Items.IndexOf(ht["cbxStopBits"].ToString());
            else
                cbxStopBits.SelectedIndex = 0;
            cbxStopBits.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion

            #region 列出数据位
            cbxDataBits.Items.Add("8");
            cbxDataBits.Items.Add("7");
            cbxDataBits.Items.Add("6");
            cbxDataBits.Items.Add("5");
            if (ht.ContainsKey("cbxDataBits"))
                cbxDataBits.SelectedIndex = cbxDataBits.Items.IndexOf(ht["cbxDataBits"].ToString());
            else
                cbxDataBits.SelectedIndex = 0;
            cbxDataBits.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion

            #region 列出奇偶校验位
            cbxParity.Items.Add("无");
            cbxParity.Items.Add("奇校验");
            cbxParity.Items.Add("偶校验");
            if (ht.ContainsKey("cbxParity"))
                cbxParity.SelectedIndex = cbxParity.Items.IndexOf(ht["cbxParity"].ToString());
            else
                cbxParity.SelectedIndex = 0;
            cbxParity.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion
            /*
            #region COM口重新加载
            cbxCOMPort.Items.Clear();//清除当前串口号中的所有串口名称
            cbxCOMPort.Items.AddRange(Methods.ActivePorts());
            if (ht.ContainsKey("cbxCOMPort") && cbxCOMPort.Items.Contains(ht["cbxCOMPort"].ToString()))
                cbxCOMPort.SelectedIndex = cbxCOMPort.Items.IndexOf(ht["cbxCOMPort"].ToString());
            else
               // cbxCOMPort.SelectedIndex = 1;
            cbxCOMPort.DropDownStyle = ComboBoxStyle.DropDownList;
            #endregion
            */
            #region 初始化计数器
            tbSendCount.Text = "0";
            tbSendCount.ReadOnly = true;
            tbReceivedCount.Text = "0";
            tbReceivedCount.ReadOnly = true;
            #endregion

            #region 初始化当前时间
            toolStripStatusLabel3.Text = DateTime.Now.ToString();
            #endregion

            #region 初始化串口状态
            toolStripStatusLabel1.ForeColor = Color.Blue;
            if (!isSetProperty)//串口未设置则设置串口属性
            {
                //cbxCOMPort.Text = "1";
                SetPortProperty();
                isSetProperty = true;
            }
            try
            {
                sp.Open();
                btnOpenCom.Text = "关闭串口";
                DisplayPortState(PortState.打开);
            }
            catch (Exception)
            {
                //串口打开失败后，串口属性设置标志位设为false
                isSetProperty = false;
                MessageBox.Show("串口无效或已被占用!", "错误提示");
            }

            #endregion

            #region 初始化间隔时间
            if (ht.ContainsKey("tbSpaceTime"))
                tbSpaceTime.Text = ht["tbSpaceTime"].ToString();
            else
                tbSpaceTime.Text = "1000";
            #endregion

            #region 初始化按16进制显示状态
            if (ht.ContainsKey("cb16Display") && ht["cb16Display"].ToString() == "True")
                cb16Display.Checked = true;
            else
                cb16Display.Checked = false;
            #endregion

            #region 初始化按16进制发送状态
            if (ht.ContainsKey("cb16Send") && ht["cb16Send"].ToString() == "True")
                cb16Send.Checked = true;
            else
                cb16Send.Checked = false;
            #endregion

            #region 初始化发送区文本
            if(ht.ContainsKey("tbSendData16") && ht.ContainsKey("tbSendDataStr"))
            {
                tbSendData16 = ht["tbSendData16"].ToString();
                tbSendDataStr = ht["tbSendDataStr"].ToString();
                if (cb16Send.Checked)
                    tbSendData.Text = ht["tbSendData16"].ToString();
                else
                    tbSendData.Text = ht["tbSendDataStr"].ToString();
            }
            #endregion
            
            tbSendData.Focus();
        }

        //显示当前时间
        private void timer1_Tick(object sender, EventArgs e)
        {
            toolStripStatusLabel3.Text = DateTime.Now.ToString();
            if (check_display.Checked == true)
            {
                tbReceivedData.Clear();
            }
        }

        //点击打开串口按钮
        private void btnOpenCom_Click(object sender, EventArgs e)
        {
            if (!sp.IsOpen)//串口没有打开时
            {
                if (!isSetProperty)//串口未设置则设置串口属性
                {
                    SetPortProperty();
                    isSetProperty = true;
                }
                try
                {
                    if(textBox2.Text != "")
                    {
                        sp.PortName = textBox2.Text.Trim();//设置波特率
                    }
                    sp.Open();
                    btnOpenCom.Text = "关闭串口";
                    DisplayPortState(PortState.打开);
                }
                catch (Exception)
                {
                    //串口打开失败后，串口属性设置标志位设为false
                    isSetProperty = false;
                    MessageBox.Show("串口无效或已被占用!", "错误提示");
                }
            }
            else//串口已经打开
            {
                try
                {
                    sp.DiscardInBuffer();//丢弃接收缓冲区数据
                    save_file_check.Checked = false;
                    sp.Close();
                    isSetProperty = false;
                    btnOpenCom.Text = "打开串口";
                    DisplayPortState(PortState.关闭);
                }
                catch (Exception)
                {
                    MessageBox.Show("关闭串口时发生错误", "错误提示");
                }
            }
        }

        //发送串口数据
        private void btnSend_Click(object sender, EventArgs e)
        {
            if (tbSendData.Text.Trim() == "")//检测发送数据是否为空
            {
                MessageBox.Show("请输入要发送的数据!", "错误提示");
                return;
            }
            if (sp.IsOpen)
            {
                DataSend();
            }
            else
            {
                MessageBox.Show("串口未打开!", "错误提示");
            }
        }

        //点击端口号选择下拉框按钮
        private void cbxCOMPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            AgainOpenPort();
        }

        //点击波特率选择下拉框按钮
        private void cbxBaudRate_SelectedIndexChanged(object sender, EventArgs e)
        {
            AgainOpenPort();
        }

        //点击数据位选择下拉框按钮
        private void cbxDataBits_SelectedIndexChanged(object sender, EventArgs e)
        {
            AgainOpenPort();
        }

        //点击停止位选择下拉框按钮
        private void cbxStopBits_SelectedIndexChanged(object sender, EventArgs e)
        {
            AgainOpenPort();
        }

        //点击校验位选择下拉框按钮
        private void cbxParity_SelectedIndexChanged(object sender, EventArgs e)
        {
            AgainOpenPort();
        }

        //点击数据接收区清空按钮
        private void btnClearReceived_Click(object sender, EventArgs e)
        {
            receivedDatas.Clear();
            tbReceivedData.Text = "";
        }

        //点击是否按16进制显示接收数据
        private void cb16Display_CheckedChanged(object sender, EventArgs e)
        {
            if (cb16Display.Checked)
                tbReceivedData.Text = Methods.ByteTo16Str(receivedDatas.ToArray());
            else
                tbReceivedData.Text = Encoding.Default.GetString(receivedDatas.ToArray());
        }

        //点击是否按16进制发送数据
        private void cb16Send_CheckedChanged(object sender, EventArgs e)
        {
            if (cb16Send.Checked)
            {
                //tbSendData.Text = tbSendData16;
                //tbSendData16 = tbSendData.Text.Trim();
            }
            else
            {
                //tbSendData.Text = tbSendDataStr;
            }
        }

        //发送文本框键盘按键检测
        private void tbSendData_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (cb16Send.Checked)
            {
                //正则匹配
                string pattern = "[0-9a-fA-F]|\b";//\b:退格键
                Match m = Regex.Match(e.KeyChar.ToString(), pattern);
                if (m.Success)
                {
                    if(e.KeyChar != '\b')
                    {
                        if (tbSendData.Text.Length % 3 == 2)
                        {
                            tbSendData.Text += " ";
                            tbSendData.SelectionStart = tbSendData.Text.Length;
                        }
                        e.KeyChar = Convert.ToChar(e.KeyChar.ToString().ToUpper());
                    }
                    e.Handled = false;
                }
                else
                {
                    e.Handled = true;
                }
            }
            else
            {
                e.Handled = false;
            }
        }

        //点击清空发送内容
        private void btnClearSend_Click(object sender, EventArgs e)
        {
            tbSendData.Text = "";
            if (cb16Send.Checked)
                tbSendData16 = "";
            else
                tbSendDataStr = "";
        }

        //点击清空计数器数据
        private void btnClearCount_Click(object sender, EventArgs e)
        {
            tbReceivedCount.Text = "0";
            tbSendCount.Text = "0";
        }

        //点击是否设置自动发送
        private void cbAutomaticSend_CheckedChanged(object sender, EventArgs e)
        {
            if (cbAutomaticSend.Checked)
            {
                timer2.Enabled = true;
                timer2.Interval = Convert.ToInt32(tbSpaceTime.Text);
            }
            else
            {
                timer2.Enabled = false;
            }
        }

        //自动发送时间文本框键盘按键检测
        private void tbSpaceTime_KeyPress(object sender, KeyPressEventArgs e)
        {
            //正则匹配
            string pattern = "[0-9]|\b";
            Match m = Regex.Match(e.KeyChar.ToString(),pattern);
            if (m.Success)
            {
                timer2.Interval = Convert.ToInt32(tbSpaceTime.Text);
                e.Handled = false;
            }
            else
            {
                e.Handled = true;
            }
        }

        //串口显示状态
        private void timer2_Tick(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                DataSend();
            }
            else
            {
                timer2.Enabled = false;
                cbAutomaticSend.Checked = false;
                MessageBox.Show("串口未打开!", "错误提示");
                
                return;
            }
            if (tbSendData.Text.Trim() == "")//检测发送数据是否为空
            {
                timer2.Enabled = false;
                cbAutomaticSend.Checked = false;
                MessageBox.Show("请输入要发送的数据!", "错误提示");
                return;
            }
        }

        //关闭窗口时出发的事件
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            try
            {
                using (StreamWriter sw = new StreamWriter(path, false))
                {
                    sw.WriteLine("cbxCOMPort=" + cbxCOMPort.Text);
                    sw.WriteLine("cbxBaudRate=" + cbxBaudRate.Text);
                    sw.WriteLine("cbxDataBits=" + cbxDataBits.Text);
                    sw.WriteLine("cbxStopBits=" + cbxStopBits.Text);
                    sw.WriteLine("cbxParity=" + cbxParity.Text);
                    sw.WriteLine("cb16Display="+ cb16Display.Checked.ToString());
                    sw.WriteLine("tbSpaceTime=" + tbSpaceTime.Text);
                    sw.WriteLine("cb16Send=" + cb16Send.Checked.ToString());
                    sw.WriteLine("tbSendDataStr=" + tbSendDataStr);
                    sw.WriteLine("tbSendData16=" + tbSendData16);
                    sp.Close();
                }
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.Message.ToString());
            }

        }

        //发送文本框按键抬起时出发的事件
        private void tbSendData_KeyUp(object sender, KeyEventArgs e)
        {
            if (cb16Send.Checked)
            {
                tbSendData16 = tbSendData.Text.Trim();
            }
            else
            {
                tbSendDataStr = tbSendData.Text;
            }
        }

        //点击读入文件按钮
        private void btnReadFile_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "所有文件(*.*)|*.*";
            //文件筛选器的设定
            openFileDialog1.FilterIndex = 1;
            openFileDialog1.Title = "选择文件";
            openFileDialog1.FileName = "";
            openFileDialog1.ShowHelp = true;
            if(openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                using (FileStream fs = new FileStream(openFileDialog1.FileName,FileMode.Open))
                {
                    byte[] bufferByte = new byte[fs.Length];
                    fs.Read(bufferByte, 0, Convert.ToInt32(fs.Length));
                    sp.Write(bufferByte, 0, bufferByte.Length);
                    tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text) + bufferByte.Length).ToString();
                }
                
            }
        }

        //点击保存数据按钮
        private void btnSaveFile_Click(object sender, EventArgs e)
        {
            

            saveFileDialog1.Filter = "所有文件(*.*)|*.*";
            if(saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                string fName = saveFileDialog1.FileName;
                using(FileStream fs = File.Open(fName, FileMode.Append))
                {
                    fs.Write(receivedDatas.ToArray(), 0, receivedDatas.Count);
                }
            }
        }

        private void tbSendData_TextChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 01";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    /*
                    if (cb16Send.Checked)                                               //十六进制
                    {
                        byte[] hexBytes = Methods._16strToHex(tbSendData16);
                        sp.Write(hexBytes, 0, hexBytes.Length);
                        tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text) + hexBytes.Length).ToString();
                    }
                    else
                    */
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 02";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();

                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button4_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    /*
                    if (cb16Send.Checked)                                               //十六进制
                    {
                        byte[] hexBytes = Methods._16strToHex(tbSendData16);
                        sp.Write(hexBytes, 0, hexBytes.Length);
                        tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text) + hexBytes.Length).ToString();
                    }
                    else
                    */
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 04";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();

                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void tbReceivedData_TextChanged(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    /*
                    if (cb16Send.Checked)                                               //十六进制
                    {
                        byte[] hexBytes = Methods._16strToHex(tbSendData16);
                        sp.Write(hexBytes, 0, hexBytes.Length);
                        tbSendCount.Text = (Convert.ToInt32(tbSendCount.Text) + hexBytes.Length).ToString();
                    }
                    else
                    */
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 03";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();

                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button5_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 05";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button6_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 06";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button8_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 07";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button7_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 08";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button9_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 11";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button10_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 12";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button12_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 13";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button11_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 14";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button13_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 15";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button14_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 16";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button16_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 17";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void button15_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                try
                {
                    tbSendData.Text = "FF A5 5A 22 06 01 13 01 02 00 18";
                    tbSendData16 = tbSendData.Text.Trim();
                    DataSend();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message.ToString());
                    return;
                }
            }
        }

        private void groupBox5_Enter(object sender, EventArgs e)
        {

        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void savefile_Click(object sender, EventArgs e)
        {

            {
                System.DateTime currentTime = new System.DateTime();
                currentTime = System.DateTime.Now;
                string strYMD = currentTime.ToString("d");
                string file_name = "";
                if (textBox1.Text == "")
                {
                    file_name = System.Convert.ToString(currentTime.Year) + '-' + System.Convert.ToString(currentTime.Month)
                 + '-' + System.Convert.ToString(currentTime.Day) + ".txt";
                    textBox1.Text = file_name;
                }
                else
                {
                    file_name = textBox1.Text + ".txt";
                }
                string newPath = System.IO.Path.Combine(".\\", file_name);
                //if (textBox1.Text != "")
                //richTextBox2.AppendText(file_name);
                //fs = new BinaryWriter(new FileStream("mydata", FileMode.Create)); //创建文件  bin目录下
                fs = File.Open(newPath, FileMode.Append);
                //fs = new FileStream(newPath, FileMode.Append);
            }
        }

        private void save_file_check_CheckedChanged(object sender, EventArgs e)
        {
            if (save_file_check.Checked == true)
            {
                System.DateTime currentTime = new System.DateTime();
                currentTime = System.DateTime.Now;
                string strYMD = currentTime.ToString("d");
                string file_name = "";
                if (textBox1.Text == "")
                {
                    if (bin_check.Checked != true)
                    {
                        file_name = System.Convert.ToString(currentTime.Year) + '-' + System.Convert.ToString(currentTime.Month)
                     + '-' + System.Convert.ToString(currentTime.Day) + ".txt";
                    }
                    else
                    {
                        file_name = System.Convert.ToString(currentTime.Year) + '-' + System.Convert.ToString(currentTime.Month)
                    + '-' + System.Convert.ToString(currentTime.Day);
                    }
                    textBox1.Text = file_name;
                }
                else
                {
                    if (bin_check.Checked != true)
                    {
                        file_name = textBox1.Text + ".txt";
                    }
                    else
                    {
                        file_name = textBox1.Text;
                    }
                }
                string newPath = System.IO.Path.Combine(".\\", file_name);
                //if (textBox1.Text != "")
                //richTextBox2.AppendText(file_name);
                textBox1.Text = newPath;
                //fs = new FileStream(newPath, FileMode.Append);
                //fs = File.Open(newPath, FileMode.Append);
            }
        }

        private void groupBox7_Enter(object sender, EventArgs e)
        {

        }

        private void tbReceivedCount_TextChanged(object sender, EventArgs e)
        {

        }

        private void groupBox4_Enter(object sender, EventArgs e)
        {

        }




    }
}
