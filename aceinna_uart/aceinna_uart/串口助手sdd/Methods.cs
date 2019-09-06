using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace aceinna_com
{
    class Methods
    {
        //获取有效的COM口
        public static string[] ActivePorts()
        {
            ArrayList activePorts = new ArrayList();
            foreach (string pname in SerialPort.GetPortNames())
            {
                activePorts.Add(Convert.ToInt32(pname.Substring(3)));
            }
            activePorts.Sort();
            string[] mystr = new string[activePorts.Count];
            int i = 0;
            foreach (int num in activePorts)
            {
                mystr[i++] = "COM" + num.ToString();
            }
            return mystr;
        }

        //16进制字符串转换为byte字符数组
        public static Byte[] _16strToHex(string strValues)
        {
            string[] hexValuesSplit = strValues.Split(' ');
            Byte[] hexValues = new Byte[hexValuesSplit.Length];
            Console.WriteLine(hexValuesSplit.Length);
            for (int i = 0; i < hexValuesSplit.Length; i++)
            {
                hexValues[i] = Convert.ToByte(hexValuesSplit[i], 16);
            }
            return hexValues;
        }

        //byte数组以16进制形式转字符串
        public static string ByteTo16Str(byte[] bytes)
        {
            string recData = null;//创建接收数据的字符串
            foreach (byte outByte in bytes)//将字节数组以16进制形式遍历到一个字符串内
            {
                recData += outByte.ToString("X2") + " ";
            }
            return recData;
        }

        //16进制字符串转换字符串
        public static string _16strToStr(string _16str)
        {
            string outStr = null;
            byte[] streamByte = _16strToHex(_16str);
            outStr = Encoding.Default.GetString(streamByte);
            return outStr;
        }
    }
}
