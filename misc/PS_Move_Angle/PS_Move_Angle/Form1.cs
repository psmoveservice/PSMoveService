using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Text.RegularExpressions;



namespace PS_Move_Angle
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Num_Degree_KeyDown(object sender, System.Windows.Forms.KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)13)
            {
                Double Degree = Convert.ToDouble(Num_Degree.Value);
                Double Angle = Math.PI * Degree / 180;
                Double WLine = Math.Cos((Angle) / 2);
                Double YLine = Math.Sin((Angle) / 2);

                lb_W.Text = Convert.ToString(Math.Round(WLine, 9).ToString("N9"));
                lb_Y.Text = Convert.ToString(Math.Round(YLine, 9).ToString("N9"));
            }
        }

        private void bt_Calc_Click(object sender, EventArgs e)
        {
            Double Degree = Convert.ToDouble(Num_Degree.Value);
            Double Angle = Math.PI * Degree / 180;
            Double WLine = Math.Cos((Angle) / 2);
            Double YLine = Math.Sin((Angle) / 2);

            lb_W.Text = Convert.ToString(Math.Round(WLine, 9).ToString("N9"));
            lb_Y.Text = Convert.ToString(Math.Round(YLine, 9).ToString("N9"));
        }

        private void bt_CopyW_Click(object sender, EventArgs e)
        {
            String WText = Convert.ToString(lb_W.Text);

            Clipboard.SetText(WText);
        }

        private void bt_CopyY_Click(object sender, EventArgs e)
        {
            String YText = Convert.ToString(lb_Y.Text);

            Clipboard.SetText(YText);
        }
    }
}
