namespace PS_Move_Angle
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.Num_Degree = new System.Windows.Forms.NumericUpDown();
            this.bt_Calc = new System.Windows.Forms.Button();
            this.bt_CopyW = new System.Windows.Forms.Button();
            this.bt_CopyY = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.panel1 = new System.Windows.Forms.Panel();
            this.lb_W = new System.Windows.Forms.Label();
            this.Label1 = new System.Windows.Forms.Label();
            this.panel2 = new System.Windows.Forms.Panel();
            this.lb_Y = new System.Windows.Forms.Label();
            this.Label2 = new System.Windows.Forms.Label();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.Num_Degree)).BeginInit();
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            this.SuspendLayout();
            // 
            // Num_Degree
            // 
            this.Num_Degree.Increment = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.Num_Degree.Location = new System.Drawing.Point(12, 12);
            this.Num_Degree.Maximum = new decimal(new int[] {
            360,
            0,
            0,
            0});
            this.Num_Degree.Name = "Num_Degree";
            this.Num_Degree.Size = new System.Drawing.Size(55, 20);
            this.Num_Degree.TabIndex = 1;
            this.toolTip1.SetToolTip(this.Num_Degree, "Input what degree you want to use.");
            this.Num_Degree.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.Num_Degree_KeyDown);
            // 
            // bt_Calc
            // 
            this.bt_Calc.Location = new System.Drawing.Point(12, 104);
            this.bt_Calc.Name = "bt_Calc";
            this.bt_Calc.Size = new System.Drawing.Size(144, 23);
            this.bt_Calc.TabIndex = 2;
            this.bt_Calc.Text = "Calculate";
            this.toolTip1.SetToolTip(this.bt_Calc, "Click to calculate \"W\" and \"Y\" from degree.");
            this.bt_Calc.UseVisualStyleBackColor = true;
            this.bt_Calc.Click += new System.EventHandler(this.bt_Calc_Click);
            // 
            // bt_CopyW
            // 
            this.bt_CopyW.Location = new System.Drawing.Point(116, 50);
            this.bt_CopyW.Name = "bt_CopyW";
            this.bt_CopyW.Size = new System.Drawing.Size(40, 23);
            this.bt_CopyW.TabIndex = 3;
            this.bt_CopyW.Text = "Copy";
            this.toolTip1.SetToolTip(this.bt_CopyW, "Click to copy \"W\" axis to clipboard");
            this.bt_CopyW.UseVisualStyleBackColor = true;
            this.bt_CopyW.Click += new System.EventHandler(this.bt_CopyW_Click);
            // 
            // bt_CopyY
            // 
            this.bt_CopyY.Location = new System.Drawing.Point(116, 75);
            this.bt_CopyY.Name = "bt_CopyY";
            this.bt_CopyY.Size = new System.Drawing.Size(40, 23);
            this.bt_CopyY.TabIndex = 4;
            this.bt_CopyY.Text = "Copy";
            this.toolTip1.SetToolTip(this.bt_CopyY, "Click to copy \"Y\" axis to clipboard");
            this.bt_CopyY.UseVisualStyleBackColor = true;
            this.bt_CopyY.Click += new System.EventHandler(this.bt_CopyY_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(73, 14);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(75, 13);
            this.label3.TabIndex = 14;
            this.label3.Text = ": Input Degree";
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.White;
            this.panel1.Controls.Add(this.lb_W);
            this.panel1.Controls.Add(this.Label1);
            this.panel1.Location = new System.Drawing.Point(12, 52);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(98, 19);
            this.panel1.TabIndex = 17;
            this.toolTip1.SetToolTip(this.panel1, "\"W\" axis");
            // 
            // lb_W
            // 
            this.lb_W.AutoSize = true;
            this.lb_W.Location = new System.Drawing.Point(28, 3);
            this.lb_W.Name = "lb_W";
            this.lb_W.Size = new System.Drawing.Size(13, 13);
            this.lb_W.TabIndex = 17;
            this.lb_W.Text = "0";
            // 
            // Label1
            // 
            this.Label1.AutoSize = true;
            this.Label1.Location = new System.Drawing.Point(4, 3);
            this.Label1.Name = "Label1";
            this.Label1.Size = new System.Drawing.Size(24, 13);
            this.Label1.TabIndex = 16;
            this.Label1.Text = "W :";
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.Color.White;
            this.panel2.Controls.Add(this.lb_Y);
            this.panel2.Controls.Add(this.Label2);
            this.panel2.Location = new System.Drawing.Point(12, 77);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(98, 19);
            this.panel2.TabIndex = 18;
            this.toolTip1.SetToolTip(this.panel2, "\"Y\" axis");
            // 
            // lb_Y
            // 
            this.lb_Y.AutoSize = true;
            this.lb_Y.Location = new System.Drawing.Point(28, 3);
            this.lb_Y.Name = "lb_Y";
            this.lb_Y.Size = new System.Drawing.Size(13, 13);
            this.lb_Y.TabIndex = 18;
            this.lb_Y.Text = "0";
            // 
            // Label2
            // 
            this.Label2.AutoSize = true;
            this.Label2.Location = new System.Drawing.Point(7, 3);
            this.Label2.Name = "Label2";
            this.Label2.Size = new System.Drawing.Size(20, 13);
            this.Label2.TabIndex = 17;
            this.Label2.Text = "Y :";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(169, 134);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.bt_CopyY);
            this.Controls.Add(this.bt_CopyW);
            this.Controls.Add(this.Num_Degree);
            this.Controls.Add(this.bt_Calc);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "Form1";
            this.ShowIcon = false;
            ((System.ComponentModel.ISupportInitialize)(this.Num_Degree)).EndInit();
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        internal System.Windows.Forms.NumericUpDown Num_Degree;
        internal System.Windows.Forms.Button bt_Calc;
        private System.Windows.Forms.Button bt_CopyW;
        private System.Windows.Forms.Button bt_CopyY;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label lb_W;
        internal System.Windows.Forms.Label Label1;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Label lb_Y;
        internal System.Windows.Forms.Label Label2;
        private System.Windows.Forms.ToolTip toolTip1;
    }
}

