using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Collections.Specialized;
using System.Xml;
using System.IO;



public class rotatetest : MonoBehaviour
{
    Vector3 k ;   //get the euler angle 
  //  Vector3 q ;  //q is the radian value of euler angle
    Vector3 t ;  //translation vector
    Vector3 t1;  //In order to compare whether the coordinate value has changed
    Vector3 q1;  //In order to compare whether the angle value has changed
    public int cam_number = 1;
    public Matrix4x4 matrix;    //extrinsic matrix
    // Start is called before the first frame update
    void Start()
    {
        
        
    }


    void Update()
    {

        GetMatrix();
    }



    //get extrinsic matrix
    private double[,] GetMatrix()
    {
        t = transform.position;//translation vector

        k = transform.eulerAngles; //get the euler angle 
      //  q = k;  //q is the radian value of k


        // convert angle to radian value
        double angle_x = Convert.ToDouble(k[0] / 180 * Math.PI);
        double angle_y = -Convert.ToDouble(k[1] / 180 * Math.PI );
        double angle_z = Convert.ToDouble(k[2] / 180 * Math.PI );

        // compute the first row of the rotation matrix
        double m00 = Convert.ToDouble(Math.Cos(angle_y) * Math.Cos(angle_z) + Math.Sin(angle_z) * Math.Sin(angle_y) * Math.Sin(angle_z));
        double m01 = Convert.ToDouble(-(Math.Cos(angle_y) * Math.Sin(angle_z)) + Math.Sin(angle_x) * Math.Sin(angle_y) * Math.Cos(angle_z));
        double m02 = Convert.ToDouble(Math.Cos(angle_x) * Math.Sin(angle_y));
        // compute the second row of the rotation matrix
        double m10 = Convert.ToDouble(Math.Cos(angle_x) * Math.Sin(angle_z));
        double m11 = Convert.ToDouble(Math.Cos(angle_x) * Math.Cos(angle_z));
        double m12 = Convert.ToDouble(-Math.Sin(angle_x));
        // compute the third row of the rotation matrix
        double m20 = Convert.ToDouble(-(Math.Sin(angle_y) * Math.Cos(angle_z)) + Math.Sin(angle_x) * Math.Cos(angle_y) * Math.Sin(angle_z));
        double m21 = Convert.ToDouble(Math.Sin(angle_y) * Math.Sin(angle_z) + Math.Sin(angle_x) * Math.Cos(angle_y) * Math.Cos(angle_z));
        double m22 = Convert.ToDouble(Math.Cos(angle_x) * Math.Cos(angle_y));

        double[,] m4m4 = new double[4, 4] { { m00, m01, m02, -t[0] },{ m10, m11, m12, t[1] } ,{ m20, m21, m22, -t[2] } ,{ 0, 0, 0, 1 } };

       /* Matrix4x4 m4x4 = new Matrix4x4(
        new Vector4(m00, m10, m20, 0),
        new Vector4(m01, m11, m21, 0),
        new Vector4(m02, m12, m22, 0),
        new Vector4(-t[0], t[1], -t[2], 1));// translation vector is 't'*/

        if (t1 != t || q1 != k)// when the position or angle of the object was changed, than show it on screen.
        {

            Debug.Log(t);
            
            Debug.Log(k);

            //Debug.Log(m4x4);

            write_to_xml(m4m4);
        }
         t1 = t;
         q1 = k;// in order to 
        return m4m4;
        
    }


    private void write_to_xml(double[,] m4x4)
    {
        if (File.Exists(Application.dataPath + "/multi-camera-results.xml"))
        {
            XmlDocument xmldoc = new XmlDocument();
            xmldoc.Load(Application.dataPath + "/multi-camera-results.xml");
            XmlNode node = xmldoc.SelectSingleNode("//opencv_storage//camera_pose_" + cam_number);
            string strname1 = node.SelectSingleNode("data").InnerText;

            //save the data to the xml file
            //string stringmatrix = m4x4.ToString();

            var s = "";
            for (int i = 0; i < m4x4.GetLength(0); i++)
            {
                for (int j = 0; j < m4x4.GetLength(1); j++)
                {
                    s += Convert.ToDouble(m4x4[i, j]).ToString("0.00000000");
                   
                    s += " ";
                }
                s += "\n";
            }

            Debug.Log("Extrinsic Matrix:");
            Debug.Log(s);
            //string stringmatrix = "abcd";
            node.SelectSingleNode("data").InnerText = s;




            XmlNode root = xmldoc.DocumentElement; 
            XmlElement xmlElem = xmldoc.DocumentElement;//get root element
            XmlNodeList bodyNode = xmlElem.GetElementsByTagName("euler_angle" + cam_number);// get bodyXmlNode
            if (bodyNode.Count > 0)
            {
                XmlNode euler_node = xmldoc.SelectSingleNode("//opencv_storage//euler_angle" + cam_number);
                var euler = "";
                euler += k[0] + " ";
                euler += k[1] + " ";
                euler += k[2];
                euler_node.InnerText = euler;
            }
            else {
                XmlElement elem = xmldoc.CreateElement("euler_angle" + cam_number);
                var euler = "";
                euler += k[0] + " ";
                euler += k[1] + " ";
                euler += k[2];

                elem.InnerText = euler;
                //Add the node to the document
                root.InsertAfter(elem, root.FirstChild);
            }


                xmldoc.Save(Application.dataPath + "/multi-camera-results.xml");

            
        }

        else
        {
            Debug.Log("NOT FOUNDED FILE");
        }
    }
}