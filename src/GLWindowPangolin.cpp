#include "GLWindowPangolin.h"

GLWindowPangolin::GLWindowPangolin(const std::string title, GS::Size sizeWindow):
    mSizeWin(sizeWindow),
    mSizeVideo(sizeWindow)
{
    pangolin::CreateWindowAndBind(title,sizeWindow.width,sizeWindow.height);//Create OpenGL window
    glEnable(GL_DEPTH_TEST);// 3D Mouse handler requires depth testing to be enabled
    mTexture.Reinitialise(mSizeVideo.width,mSizeVideo.height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
}

void GLWindowPangolin::SetupViewport()
{
    glViewport(0, 0, mSizeWin.width,mSizeWin.height);
}

void GLWindowPangolin::SetupVideoOrtho()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5,(double)mSizeVideo.width-0.5, (double)mSizeVideo.height-0.5, -0.5, -1.0, 1.0);
}

void GLWindowPangolin::SetupVideoRasterPosAndZoom()
{
    glRasterPos2d(-0.5,-0.5);
    double adZoom[2]={1.0};
    adZoom[0] = (double) mSizeWin.width / (double) mSizeVideo.width;
    adZoom[1] = (double) mSizeWin.height / (double) mSizeVideo.height;
    glPixelZoom(adZoom[0], -adZoom[1]);
}

bool GLWindowPangolin::LoadTextures(cv::Mat matImg, GLuint texture)
{
    int Status=false;
    if(!matImg.empty())
    {
        Status=true;
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, 3,
                     matImg.cols, matImg.rows,
                     0, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char *)matImg.data);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    }
    return Status;
}

void GLWindowPangolin::DrawTexture2DGray(const cv::Mat &imgGray)
{
    mTexture.Upload(imgGray.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);//upload image to GPU
}

void GLWindowPangolin::DrawTexture2DRGB(const cv::Mat &imgRGB)
{
    mTexture.Upload(imgRGB.data,GL_RGB,GL_UNSIGNED_BYTE);//upload image to GPU
}

void GLWindowPangolin::RenderTextureToViewport()
{
    glColor3f(1.0,1.0,1.0);
    mTexture.RenderToViewport(true);
}

void GLWindowPangolin::DrawPoints2D(const std::vector<cv::Point2i> &points, GS::RGB rgb, float size)
{
    if(points.empty())
    {
        return;
    }
    glColor3f(rgb.r, rgb.g, rgb.b);
    glPointSize(size);
    glBegin(GL_POINTS);
    for(unsigned int i=0; i<points.size(); i++)
        glVertex2f(points[i].x,points[i].y);
    glEnd();
}

void GLWindowPangolin::DrawLines(
        const cv::Point2i &ptStart, GS::RGB rgbStart,
        const cv::Point2i &ptEnd  , GS::RGB rgbEnd,
        float width)
{
    glLineWidth(width);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    glBegin(GL_LINES);

    glColor3f(rgbStart.r, rgbStart.g, rgbStart.b);
    glVertex2f(ptStart.x, ptStart.y);

    glColor3f(rgbEnd.r, rgbEnd.g, rgbEnd.b);
    glVertex2f(ptEnd.x, ptEnd.y);

    glEnd();
}

void GLWindowPangolin::DrawOrigeAxis()
{
    glLineWidth(3);   
    glBegin ( GL_LINES );
    glColor3f ( 0.8f,0.f,0.f );
    glVertex3f( 0,0,0 );
    glVertex3f( 10,0,0 );
    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,10,0 );
    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,10 );
    glEnd();
}


