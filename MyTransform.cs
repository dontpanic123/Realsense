using UnityEngine;
using System.IO;//If you want to use the FIle class and FileStream class, you have to use this namespace
                //IO: Input / Output
using System;
using System.Xml;
//using OpenCvSharp;

public class MyTransform : MonoBehaviour
{
    public Matrix4x4 matrix;    //extrinsic matrix
    public Vector4 v;           //load vector
    public Vector4 startPos;    //start position
    public float x = 1;         //translation variable
    public int cam_number = 1;
    //rotation variable
    public float angle;
    
   
    private void Start()
    {
        LoadByXML();
        
       
        //startPos = new Vector4(transform.position.x, transform.position.y, transform.position.z, 1);
        //matrix.SetTRS(transform.position, transform.rotation, transform.localScale);
        //MyScale();
    }

   
    private void Update()
    {
        //MyTranslate();
    }


    public void LoadByXML()
    {
        // Debug.Log(Application.dataPath);
        if (File.Exists(Application.dataPath + "/multi-camera-results.xml"))
        {
            //LOAD THE DATA
            Save save = new Save();

            XmlDocument xmlDocument = new XmlDocument();
            xmlDocument.Load(Application.dataPath + "/multi-camera-results.xml");

            //MARKER Get the SAVE FILE DATA from the FILE
            //XmlNodeList: Represents an ordered collection of nodes.
            XmlNodeList NCameras = xmlDocument.GetElementsByTagName("nCameras");
            int camerasNumCount = int.Parse(NCameras[0].InnerText);
            save.nCameras = camerasNumCount;
            Debug.Log("The number of camera is:");
            Debug.Log(camerasNumCount);

            XmlNodeList reprojectionserror = xmlDocument.GetElementsByTagName("meanReprojectError");
            float Reprojection = float.Parse(reprojectionserror[0].InnerText);
            save.reprojection = Reprojection;
            Debug.Log("The Reprojection error is:");
            Debug.Log(Reprojection);



            /*  XmlNodeList row = xmlDocument.GetElementsByTagName("rows");
              int ROW = int.Parse(row[2].InnerText);
              save.Row = ROW;
              Debug.Log(ROW);

              XmlNodeList col = xmlDocument.GetElementsByTagName("cols");
              int COL = int.Parse(col[2].InnerText);
              save.Col = COL;
              Debug.Log(COL);*/



            XmlDocument xmldoc = new XmlDocument();
            xmldoc.Load(Application.dataPath + "/multi-camera-results.xml");
            XmlNode node = xmldoc.SelectSingleNode("//opencv_storage//camera_pose_"+cam_number);
            string strname1 = node.SelectSingleNode("data").InnerText;
            string[] strArray = strname1.Split(new char[4] { ' ', '\n', '\r', '	' }, StringSplitOptions.RemoveEmptyEntries);
            double[] strArrayDouble = new double[strArray.Length];
            Debug.Log("Extrinsic Matrix:");
            for (int i = 0; i < strArray.Length; i++)
            {
                strArrayDouble[i] = Convert.ToDouble(strArray[i].ToString());
                string j = strArray[i].ToString();
                Debug.Log(j);
            }

            //translation vector
            matrix = Matrix4x4.identity; //unit matrix
            matrix.m03 =(float)(Convert.ToDouble(strArray[3].ToString())); //x axial
            matrix.m13 = (float)(Convert.ToDouble(strArray[7].ToString())); //y axial
            matrix.m23 = (float)(Convert.ToDouble(strArray[11].ToString())); //z axial


            //Scale matrix
            /*
            matrix.m00 = (float)(1.0); //x axial
            matrix.m11 = (float)(2.0); //y axial
            matrix.m22 = (float)(3.0); //z axial
            */

            //rotation matrix
            
            matrix.m00 = (float)(Convert.ToDouble(strArray[0].ToString())); 
            matrix.m01 = (float)(Convert.ToDouble(strArray[1].ToString())); 
            matrix.m02 = (float)(Convert.ToDouble(strArray[2].ToString())); 
            matrix.m10 = (float)(Convert.ToDouble(strArray[4].ToString())); 
            matrix.m11 = (float)(Convert.ToDouble(strArray[5].ToString())); 
            matrix.m12 = (float)(Convert.ToDouble(strArray[6].ToString())); 
            matrix.m20 = (float)(Convert.ToDouble(strArray[8].ToString())); 
            matrix.m21 = (float)(Convert.ToDouble(strArray[9].ToString())); 
            matrix.m22 = (float)(Convert.ToDouble(strArray[10].ToString()));


            XmlElement xmlElem = xmldoc.DocumentElement;//get root element
            XmlNodeList bodyNode = xmlElem.GetElementsByTagName("euler_angle" + cam_number);// get bodyXmlNode
            if (bodyNode.Count > 0)
            {

                Rotation_euler();
                MyTranslate();
            }
            else
            {
                MyRotation();
                MyTranslate();
            }
        }


        else
        {
            Debug.Log("NOT FOUNDED FILE");
            
        }


    }

    void MyTranslate()
    {
        v = new Vector4(transform.position.x, transform.position.y, transform.position.z, 1);
        //v = matrix * startPos;
        v = matrix * v;
        transform.position = new Vector3(-v.x, v.y, -v.z);
        
    }

    void MyScale()
    {
        matrix = Matrix4x4.identity;
        v = new Vector4(transform.localScale.x, transform.localScale.y, transform.localScale.z, 1);
        matrix.m00 = (float)(1.0); //x axial
        matrix.m11 = (float)(2.0); //y axial
        matrix.m22 = (float)(3.0); //z axial
        v = matrix * v;
        transform.localScale = new Vector3(v.x, v.y, v.z);
    }

    void MyRotation() {


        //transfor rotation matrix to quaternion
        float qw = Mathf.Sqrt(1f + matrix.m00 + matrix.m11 + matrix.m22) / 2;
        float w = 4 * qw;
        float qx = (matrix.m21 - matrix.m12) / w;
        float qy = (matrix.m02 - matrix.m20) / w;
        float qz = (matrix.m10 - matrix.m01) / w;

        transform.rotation = new Quaternion(qx, -qy, qz, qw);

        /* 
    float qx = 180*(Mathf.Asin(-matrix.m12)/ 3.141592);
    float qy = 180*(Mathf.Atan(matrix.m02 / matrix.m22)/ 3.14159);
    float qz = 180*(Mathf.Atan(matrix.m10 / matrix.m11)/ 3.14159);

    transform.localEulerAngles = new Vector3(qx,qy,qz);*/
    }
    void Rotation_euler() {
        XmlDocument xmldoc1 = new XmlDocument();
        xmldoc1.Load(Application.dataPath + "/multi-camera-results.xml");
        XmlNode node_euler = xmldoc1.SelectSingleNode("//opencv_storage//euler_angle" + cam_number);
        string strname2 = node_euler.InnerText;
        string[] strArray = strname2.Split(new char[4] { ' ', '\n', '\r', '	' }, StringSplitOptions.RemoveEmptyEntries);
        float[] strArrayFloat = new float[strArray.Length];
        Debug.Log("Euler angle:");
        for (int i = 0; i < strArray.Length; i++)
        {
            strArrayFloat[i] = Convert.ToSingle(strArray[i].ToString());
            string j = strArray[i].ToString();
            Debug.Log(j);
        }
        transform.localEulerAngles = new Vector3(strArrayFloat[0], strArrayFloat[1], strArrayFloat[2]);

    }
}
