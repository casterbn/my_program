namespace aceinna_com
{
    partial class Form1
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.btnOpenCom = new System.Windows.Forms.Button();
            this.cbxParity = new System.Windows.Forms.ComboBox();
            this.cbxStopBits = new System.Windows.Forms.ComboBox();
            this.cbxDataBits = new System.Windows.Forms.ComboBox();
            this.cbxBaudRate = new System.Windows.Forms.ComboBox();
            this.cbxCOMPort = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.btnClearCount = new System.Windows.Forms.Button();
            this.tbReceivedCount = new System.Windows.Forms.TextBox();
            this.tbSendCount = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.check_display = new System.Windows.Forms.CheckBox();
            this.btnSaveFile = new System.Windows.Forms.Button();
            this.btnClearReceived = new System.Windows.Forms.Button();
            this.cb16Display = new System.Windows.Forms.CheckBox();
            this.tbReceivedData = new System.Windows.Forms.TextBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.btnReadFile = new System.Windows.Forms.Button();
            this.cb16Send = new System.Windows.Forms.CheckBox();
            this.cks_text = new System.Windows.Forms.TextBox();
            this.btnSend = new System.Windows.Forms.Button();
            this.btnClearSend = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.tbSpaceTime = new System.Windows.Forms.TextBox();
            this.cbAutomaticSend = new System.Windows.Forms.CheckBox();
            this.tbSendData = new System.Windows.Forms.TextBox();
            this.statusStrip1 = new System.Windows.Forms.StatusStrip();
            this.toolStripStatusLabel1 = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabel2 = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabel3 = new System.Windows.Forms.ToolStripStatusLabel();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.timer2 = new System.Windows.Forms.Timer(this.components);
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.saveFileDialog1 = new System.Windows.Forms.SaveFileDialog();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.bin_check = new System.Windows.Forms.CheckBox();
            this.save_file_check = new System.Windows.Forms.CheckBox();
            this.savefile1 = new System.Windows.Forms.Button();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.statusStrip1.SuspendLayout();
            this.groupBox7.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.textBox2);
            this.groupBox1.Controls.Add(this.btnOpenCom);
            this.groupBox1.Controls.Add(this.cbxParity);
            this.groupBox1.Controls.Add(this.cbxStopBits);
            this.groupBox1.Controls.Add(this.cbxDataBits);
            this.groupBox1.Controls.Add(this.cbxBaudRate);
            this.groupBox1.Controls.Add(this.cbxCOMPort);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Location = new System.Drawing.Point(18, 18);
            this.groupBox1.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox1.Size = new System.Drawing.Size(252, 276);
            this.groupBox1.TabIndex = 0;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "初始化";
            // 
            // textBox2
            // 
            this.textBox2.Location = new System.Drawing.Point(141, 236);
            this.textBox2.Name = "textBox2";
            this.textBox2.Size = new System.Drawing.Size(100, 28);
            this.textBox2.TabIndex = 11;
            // 
            // btnOpenCom
            // 
            this.btnOpenCom.Location = new System.Drawing.Point(22, 236);
            this.btnOpenCom.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnOpenCom.Name = "btnOpenCom";
            this.btnOpenCom.Size = new System.Drawing.Size(112, 28);
            this.btnOpenCom.TabIndex = 10;
            this.btnOpenCom.Text = "打开串口";
            this.btnOpenCom.UseVisualStyleBackColor = true;
            this.btnOpenCom.Click += new System.EventHandler(this.btnOpenCom_Click);
            // 
            // cbxParity
            // 
            this.cbxParity.FormattingEnabled = true;
            this.cbxParity.Location = new System.Drawing.Point(112, 185);
            this.cbxParity.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbxParity.Name = "cbxParity";
            this.cbxParity.Size = new System.Drawing.Size(118, 26);
            this.cbxParity.TabIndex = 9;
            this.cbxParity.SelectedIndexChanged += new System.EventHandler(this.cbxParity_SelectedIndexChanged);
            // 
            // cbxStopBits
            // 
            this.cbxStopBits.FormattingEnabled = true;
            this.cbxStopBits.Location = new System.Drawing.Point(112, 146);
            this.cbxStopBits.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbxStopBits.Name = "cbxStopBits";
            this.cbxStopBits.Size = new System.Drawing.Size(118, 26);
            this.cbxStopBits.TabIndex = 8;
            this.cbxStopBits.SelectedIndexChanged += new System.EventHandler(this.cbxStopBits_SelectedIndexChanged);
            // 
            // cbxDataBits
            // 
            this.cbxDataBits.FormattingEnabled = true;
            this.cbxDataBits.Location = new System.Drawing.Point(112, 109);
            this.cbxDataBits.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbxDataBits.Name = "cbxDataBits";
            this.cbxDataBits.Size = new System.Drawing.Size(118, 26);
            this.cbxDataBits.TabIndex = 7;
            this.cbxDataBits.SelectedIndexChanged += new System.EventHandler(this.cbxDataBits_SelectedIndexChanged);
            // 
            // cbxBaudRate
            // 
            this.cbxBaudRate.FormattingEnabled = true;
            this.cbxBaudRate.Location = new System.Drawing.Point(112, 73);
            this.cbxBaudRate.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbxBaudRate.Name = "cbxBaudRate";
            this.cbxBaudRate.Size = new System.Drawing.Size(118, 26);
            this.cbxBaudRate.TabIndex = 6;
            this.cbxBaudRate.SelectedIndexChanged += new System.EventHandler(this.cbxBaudRate_SelectedIndexChanged);
            // 
            // cbxCOMPort
            // 
            this.cbxCOMPort.FormattingEnabled = true;
            this.cbxCOMPort.Location = new System.Drawing.Point(112, 35);
            this.cbxCOMPort.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbxCOMPort.Name = "cbxCOMPort";
            this.cbxCOMPort.Size = new System.Drawing.Size(118, 26);
            this.cbxCOMPort.TabIndex = 5;
            this.cbxCOMPort.Text = "COM1";
            this.cbxCOMPort.SelectedIndexChanged += new System.EventHandler(this.cbxCOMPort_SelectedIndexChanged);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(19, 190);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(62, 18);
            this.label5.TabIndex = 4;
            this.label5.Text = "校验位";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(19, 151);
            this.label4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(62, 18);
            this.label4.TabIndex = 3;
            this.label4.Text = "停止位";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(19, 114);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(62, 18);
            this.label3.TabIndex = 2;
            this.label3.Text = "数据位";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(19, 77);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(62, 18);
            this.label2.TabIndex = 1;
            this.label2.Text = "波特率";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(19, 38);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(62, 18);
            this.label1.TabIndex = 0;
            this.label1.Text = "端口号";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.btnClearCount);
            this.groupBox2.Controls.Add(this.tbReceivedCount);
            this.groupBox2.Controls.Add(this.tbSendCount);
            this.groupBox2.Controls.Add(this.label8);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Location = new System.Drawing.Point(18, 394);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox2.Size = new System.Drawing.Size(251, 167);
            this.groupBox2.TabIndex = 1;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "计数";
            // 
            // btnClearCount
            // 
            this.btnClearCount.Location = new System.Drawing.Point(62, 115);
            this.btnClearCount.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnClearCount.Name = "btnClearCount";
            this.btnClearCount.Size = new System.Drawing.Size(112, 35);
            this.btnClearCount.TabIndex = 4;
            this.btnClearCount.Text = "清空计数";
            this.btnClearCount.UseVisualStyleBackColor = true;
            this.btnClearCount.Click += new System.EventHandler(this.btnClearCount_Click);
            // 
            // tbReceivedCount
            // 
            this.tbReceivedCount.BackColor = System.Drawing.Color.White;
            this.tbReceivedCount.Location = new System.Drawing.Point(117, 74);
            this.tbReceivedCount.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tbReceivedCount.Name = "tbReceivedCount";
            this.tbReceivedCount.Size = new System.Drawing.Size(118, 28);
            this.tbReceivedCount.TabIndex = 3;
            this.tbReceivedCount.TextChanged += new System.EventHandler(this.tbReceivedCount_TextChanged);
            // 
            // tbSendCount
            // 
            this.tbSendCount.BackColor = System.Drawing.Color.White;
            this.tbSendCount.Location = new System.Drawing.Point(118, 38);
            this.tbSendCount.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tbSendCount.Name = "tbSendCount";
            this.tbSendCount.Size = new System.Drawing.Size(118, 28);
            this.tbSendCount.TabIndex = 2;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(12, 79);
            this.label8.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(98, 18);
            this.label8.TabIndex = 1;
            this.label8.Text = "接收(byte)";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(12, 43);
            this.label7.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(98, 18);
            this.label7.TabIndex = 0;
            this.label7.Text = "发送(byte)";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.check_display);
            this.groupBox3.Controls.Add(this.btnSaveFile);
            this.groupBox3.Controls.Add(this.btnClearReceived);
            this.groupBox3.Controls.Add(this.cb16Display);
            this.groupBox3.Location = new System.Drawing.Point(279, 18);
            this.groupBox3.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox3.Size = new System.Drawing.Size(558, 366);
            this.groupBox3.TabIndex = 2;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "数据接收区";
            // 
            // check_display
            // 
            this.check_display.AutoSize = true;
            this.check_display.Location = new System.Drawing.Point(8, 296);
            this.check_display.Name = "check_display";
            this.check_display.Size = new System.Drawing.Size(70, 22);
            this.check_display.TabIndex = 4;
            this.check_display.Text = "显示";
            this.check_display.UseVisualStyleBackColor = true;
            // 
            // btnSaveFile
            // 
            this.btnSaveFile.Location = new System.Drawing.Point(418, 320);
            this.btnSaveFile.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnSaveFile.Name = "btnSaveFile";
            this.btnSaveFile.Size = new System.Drawing.Size(112, 35);
            this.btnSaveFile.TabIndex = 3;
            this.btnSaveFile.Text = "数据保存";
            this.btnSaveFile.UseVisualStyleBackColor = true;
            this.btnSaveFile.Click += new System.EventHandler(this.btnSaveFile_Click);
            // 
            // btnClearReceived
            // 
            this.btnClearReceived.Location = new System.Drawing.Point(280, 318);
            this.btnClearReceived.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnClearReceived.Name = "btnClearReceived";
            this.btnClearReceived.Size = new System.Drawing.Size(112, 35);
            this.btnClearReceived.TabIndex = 2;
            this.btnClearReceived.Text = "清空内容";
            this.btnClearReceived.UseVisualStyleBackColor = true;
            this.btnClearReceived.Click += new System.EventHandler(this.btnClearReceived_Click);
            // 
            // cb16Display
            // 
            this.cb16Display.AutoSize = true;
            this.cb16Display.Location = new System.Drawing.Point(9, 326);
            this.cb16Display.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cb16Display.Name = "cb16Display";
            this.cb16Display.Size = new System.Drawing.Size(142, 22);
            this.cb16Display.TabIndex = 1;
            this.cb16Display.Text = "按16进制显示";
            this.cb16Display.UseVisualStyleBackColor = true;
            this.cb16Display.CheckedChanged += new System.EventHandler(this.cb16Display_CheckedChanged);
            // 
            // tbReceivedData
            // 
            this.tbReceivedData.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.tbReceivedData.Location = new System.Drawing.Point(288, 30);
            this.tbReceivedData.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tbReceivedData.Multiline = true;
            this.tbReceivedData.Name = "tbReceivedData";
            this.tbReceivedData.ReadOnly = true;
            this.tbReceivedData.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbReceivedData.Size = new System.Drawing.Size(644, 278);
            this.tbReceivedData.TabIndex = 0;
            this.tbReceivedData.TextChanged += new System.EventHandler(this.tbReceivedData_TextChanged);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.btnReadFile);
            this.groupBox4.Controls.Add(this.cb16Send);
            this.groupBox4.Controls.Add(this.cks_text);
            this.groupBox4.Controls.Add(this.btnSend);
            this.groupBox4.Controls.Add(this.btnClearSend);
            this.groupBox4.Controls.Add(this.label9);
            this.groupBox4.Controls.Add(this.tbSpaceTime);
            this.groupBox4.Controls.Add(this.cbAutomaticSend);
            this.groupBox4.Controls.Add(this.tbSendData);
            this.groupBox4.Location = new System.Drawing.Point(278, 394);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox4.Size = new System.Drawing.Size(655, 214);
            this.groupBox4.TabIndex = 3;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "数据发送区";
            this.groupBox4.Enter += new System.EventHandler(this.groupBox4_Enter);
            // 
            // btnReadFile
            // 
            this.btnReadFile.Location = new System.Drawing.Point(282, 172);
            this.btnReadFile.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnReadFile.Name = "btnReadFile";
            this.btnReadFile.Size = new System.Drawing.Size(112, 35);
            this.btnReadFile.TabIndex = 7;
            this.btnReadFile.Text = "读入文件";
            this.btnReadFile.UseVisualStyleBackColor = true;
            this.btnReadFile.Click += new System.EventHandler(this.btnReadFile_Click);
            // 
            // cb16Send
            // 
            this.cb16Send.AutoSize = true;
            this.cb16Send.Location = new System.Drawing.Point(10, 173);
            this.cb16Send.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cb16Send.Name = "cb16Send";
            this.cb16Send.Size = new System.Drawing.Size(142, 22);
            this.cb16Send.TabIndex = 6;
            this.cb16Send.Text = "按16进制发送";
            this.cb16Send.UseVisualStyleBackColor = true;
            this.cb16Send.CheckedChanged += new System.EventHandler(this.cb16Send_CheckedChanged);
            // 
            // cks_text
            // 
            this.cks_text.Location = new System.Drawing.Point(404, 136);
            this.cks_text.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.cks_text.Name = "cks_text";
            this.cks_text.Size = new System.Drawing.Size(112, 28);
            this.cks_text.TabIndex = 9;
            // 
            // btnSend
            // 
            this.btnSend.Location = new System.Drawing.Point(404, 172);
            this.btnSend.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(112, 35);
            this.btnSend.TabIndex = 5;
            this.btnSend.Text = "发送";
            this.btnSend.UseVisualStyleBackColor = true;
            this.btnSend.Click += new System.EventHandler(this.btnSend_Click);
            // 
            // btnClearSend
            // 
            this.btnClearSend.Location = new System.Drawing.Point(282, 136);
            this.btnClearSend.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.btnClearSend.Name = "btnClearSend";
            this.btnClearSend.Size = new System.Drawing.Size(112, 35);
            this.btnClearSend.TabIndex = 4;
            this.btnClearSend.Text = "清空内容";
            this.btnClearSend.UseVisualStyleBackColor = true;
            this.btnClearSend.Click += new System.EventHandler(this.btnClearSend_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(233, 145);
            this.label9.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(26, 18);
            this.label9.TabIndex = 3;
            this.label9.Text = "ms";
            // 
            // tbSpaceTime
            // 
            this.tbSpaceTime.Location = new System.Drawing.Point(170, 136);
            this.tbSpaceTime.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tbSpaceTime.Name = "tbSpaceTime";
            this.tbSpaceTime.Size = new System.Drawing.Size(58, 28);
            this.tbSpaceTime.TabIndex = 2;
            this.tbSpaceTime.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tbSpaceTime_KeyPress);
            // 
            // cbAutomaticSend
            // 
            this.cbAutomaticSend.AutoSize = true;
            this.cbAutomaticSend.Location = new System.Drawing.Point(10, 138);
            this.cbAutomaticSend.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.cbAutomaticSend.Name = "cbAutomaticSend";
            this.cbAutomaticSend.Size = new System.Drawing.Size(160, 22);
            this.cbAutomaticSend.TabIndex = 1;
            this.cbAutomaticSend.Text = "自动发送：间隔";
            this.cbAutomaticSend.UseVisualStyleBackColor = true;
            this.cbAutomaticSend.CheckedChanged += new System.EventHandler(this.cbAutomaticSend_CheckedChanged);
            // 
            // tbSendData
            // 
            this.tbSendData.Location = new System.Drawing.Point(10, 24);
            this.tbSendData.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tbSendData.Multiline = true;
            this.tbSendData.Name = "tbSendData";
            this.tbSendData.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbSendData.Size = new System.Drawing.Size(644, 101);
            this.tbSendData.TabIndex = 0;
            this.tbSendData.TextChanged += new System.EventHandler(this.tbSendData_TextChanged);
            this.tbSendData.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tbSendData_KeyPress);
            this.tbSendData.KeyUp += new System.Windows.Forms.KeyEventHandler(this.tbSendData_KeyUp);
            // 
            // statusStrip1
            // 
            this.statusStrip1.ImageScalingSize = new System.Drawing.Size(20, 20);
            this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripStatusLabel1,
            this.toolStripStatusLabel2,
            this.toolStripStatusLabel3});
            this.statusStrip1.Location = new System.Drawing.Point(0, 635);
            this.statusStrip1.Name = "statusStrip1";
            this.statusStrip1.Padding = new System.Windows.Forms.Padding(1, 0, 21, 0);
            this.statusStrip1.Size = new System.Drawing.Size(1284, 29);
            this.statusStrip1.TabIndex = 4;
            this.statusStrip1.Text = "statusStrip1";
            // 
            // toolStripStatusLabel1
            // 
            this.toolStripStatusLabel1.Name = "toolStripStatusLabel1";
            this.toolStripStatusLabel1.Size = new System.Drawing.Size(195, 24);
            this.toolStripStatusLabel1.Text = "toolStripStatusLabel1";
            // 
            // toolStripStatusLabel2
            // 
            this.toolStripStatusLabel2.Name = "toolStripStatusLabel2";
            this.toolStripStatusLabel2.Size = new System.Drawing.Size(872, 24);
            this.toolStripStatusLabel2.Spring = true;
            // 
            // toolStripStatusLabel3
            // 
            this.toolStripStatusLabel3.Name = "toolStripStatusLabel3";
            this.toolStripStatusLabel3.Size = new System.Drawing.Size(195, 24);
            this.toolStripStatusLabel3.Text = "toolStripStatusLabel3";
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 1000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // timer2
            // 
            this.timer2.Tick += new System.EventHandler(this.timer2_Tick);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // groupBox5
            // 
            this.groupBox5.Location = new System.Drawing.Point(979, 30);
            this.groupBox5.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox5.Size = new System.Drawing.Size(125, 200);
            this.groupBox5.TabIndex = 10;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "控制模块1";
            this.groupBox5.Enter += new System.EventHandler(this.groupBox5_Enter);
            // 
            // groupBox6
            // 
            this.groupBox6.Location = new System.Drawing.Point(1126, 30);
            this.groupBox6.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox6.Size = new System.Drawing.Size(124, 200);
            this.groupBox6.TabIndex = 11;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "控制模块2";
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.bin_check);
            this.groupBox7.Controls.Add(this.save_file_check);
            this.groupBox7.Controls.Add(this.savefile1);
            this.groupBox7.Controls.Add(this.textBox1);
            this.groupBox7.Controls.Add(this.label6);
            this.groupBox7.Location = new System.Drawing.Point(979, 241);
            this.groupBox7.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox7.Size = new System.Drawing.Size(271, 200);
            this.groupBox7.TabIndex = 13;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "控制模块3";
            this.groupBox7.Enter += new System.EventHandler(this.groupBox7_Enter);
            // 
            // bin_check
            // 
            this.bin_check.AutoSize = true;
            this.bin_check.Location = new System.Drawing.Point(114, 142);
            this.bin_check.Name = "bin_check";
            this.bin_check.Size = new System.Drawing.Size(124, 22);
            this.bin_check.TabIndex = 4;
            this.bin_check.Text = "二进制保存";
            this.bin_check.UseVisualStyleBackColor = true;
            // 
            // save_file_check
            // 
            this.save_file_check.AutoSize = true;
            this.save_file_check.Location = new System.Drawing.Point(10, 142);
            this.save_file_check.Name = "save_file_check";
            this.save_file_check.Size = new System.Drawing.Size(70, 22);
            this.save_file_check.TabIndex = 3;
            this.save_file_check.Text = "保存";
            this.save_file_check.UseVisualStyleBackColor = true;
            this.save_file_check.CheckedChanged += new System.EventHandler(this.save_file_check_CheckedChanged);
            // 
            // savefile1
            // 
            this.savefile1.Location = new System.Drawing.Point(0, 95);
            this.savefile1.Name = "savefile1";
            this.savefile1.Size = new System.Drawing.Size(86, 26);
            this.savefile1.TabIndex = 2;
            this.savefile1.Text = "新建";
            this.savefile1.UseVisualStyleBackColor = true;
            this.savefile1.Click += new System.EventHandler(this.savefile_Click);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(92, 45);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(173, 28);
            this.textBox1.TabIndex = 1;
            this.textBox1.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(7, 48);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(62, 18);
            this.label6.TabIndex = 0;
            this.label6.Text = "文件名";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Silver;
            this.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.ClientSize = new System.Drawing.Size(1284, 664);
            this.Controls.Add(this.groupBox7);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.tbReceivedData);
            this.Controls.Add(this.statusStrip1);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Cursor = System.Windows.Forms.Cursors.AppStarting;
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "Form1";
            this.Text = "aceinna_com\r\n by dch";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Form1_FormClosed);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.statusStrip1.ResumeLayout(false);
            this.statusStrip1.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.ComboBox cbxParity;
        private System.Windows.Forms.ComboBox cbxStopBits;
        private System.Windows.Forms.ComboBox cbxDataBits;
        private System.Windows.Forms.ComboBox cbxBaudRate;
        private System.Windows.Forms.ComboBox cbxCOMPort;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnOpenCom;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox tbReceivedCount;
        private System.Windows.Forms.TextBox tbSendCount;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Button btnClearCount;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button btnClearReceived;
        private System.Windows.Forms.CheckBox cb16Display;
        private System.Windows.Forms.TextBox tbReceivedData;
        private System.Windows.Forms.TextBox tbSpaceTime;
        private System.Windows.Forms.CheckBox cbAutomaticSend;
        private System.Windows.Forms.TextBox tbSendData;
        private System.Windows.Forms.Button btnSend;
        private System.Windows.Forms.Button btnClearSend;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.StatusStrip statusStrip1;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel1;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel2;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel3;
        private System.Windows.Forms.CheckBox cb16Send;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Button btnSaveFile;
        private System.Windows.Forms.Button btnReadFile;
        private System.Windows.Forms.Timer timer2;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.SaveFileDialog saveFileDialog1;
        private System.Windows.Forms.TextBox cks_text;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button savefile1;
        private System.Windows.Forms.CheckBox save_file_check;
        private System.Windows.Forms.CheckBox check_display;
        private System.Windows.Forms.CheckBox bin_check;
        private System.Windows.Forms.TextBox textBox2;
    }
}

