#ifndef _SFM_GSOC2011_MACRO_H_
#define _SFM_GSOC2011_MACRO_H_

//Taken from opencv's precom.hpp
#if _MSC_VER >= 1200
#pragma warning( disable: 4127 4251 4521 4996 )
#endif

#if ( defined WIN32 || defined _WIN32 || defined WINCE )
  #if defined SFM_API_EXPORTS
      #define SFM_EXPORTS __declspec( dllexport )
  #else
      #define SFM_EXPORTS __declspec( dllimport )
  #endif
#else
    #define SFM_EXPORTS
#endif


/*! \mainpage SDK Structure From Motion Documentation
*
* \section Intro What's the point?
*\htmlonly
<div style="width:600px;">
 <img alt="Structure from motion" src="http://charlie-soft.com/Structure%20from%20motion/opencvLogoFinal.png" align="left" width="100px"/>
 <p><img alt="Structure from motion" src="http://charlie-soft.com/Structure%20from%20motion/degrad.png"  align="right"/>
 Structure from motion aims to find both cameras and objects position, orientation and shape.<br/>
 As this task is complex and highly depends on videos contents, a fast-robust-accurate technic who works with every types of input is still a dream. This API try to give to user an easy way to try differents algorithms for points detection, matching and of course geometry recovery.<br/>
 So in a long term, you will be able to do, with this API:
 </p>
 <ul>
 <li>Manage one or several cameras (I mean physical device) in a sequence (stereovision, single camera, multivision...).</li>
 <li>Each camera can be of different type (Fisheye, with/without radial distortion, various intra-parameters...).</li>
 <li>Initialize the different processing blocks according to the data availables:
 <ul><li>Camera: Distortion, intra parameters, nothing, ...</li>
     <li>Field of view: Extern position, points of interest, a known 3D pattern to match, 2D images, ...</li>
 </ul></li>
 <li>Compute missing data (intra/extern parameters, 3D points,...)</li>
 <li>Show the points cloud using an interactive visualization</li>
 </ul>

 \endhtmlonly
 \latexonly
 Structure from motion aims to find both cameras and objects position, orientation and shape.\\
 As this task is complex and highly depends on videos contents, a fast-robust-accurate technic who works with every types of input is still a dream. This API try to give to user an easy way to try differents algorithms for points detection, matching and of course geometry recovery.\\
 So in a long term, you will be able to do, with this API:
 \begin{itemize}
 \item Manage one or several cameras (I mean physical device) in a sequence (stereovision, single camera, multivision...).
 \item Each camera can be of different type (Fisheye, with/without radial distortion, various intra-parameters...).
 \item Initialize the different processing blocks according to the data availables\\
 Camera: Distortion, intra parameters, nothing, \dots \\
 Field of view: Extern position, points of interest, a known 3D pattern to match, 2D images, \dots
 \item Compute missing data (intra/extern parameters, 3D points,...)
 \item Show the points cloud using an interactive visualization
 \end{itemize}
 \endlatexonly
 * \section Example Example
 *\htmlonly

 I made a little video to show current reconstruction progress. This is not really a structure from motion as the cameras are fully parameterized, but it's a start:<br/><br/>
 <iframe width="420" height="345" src="http://www.youtube.com/embed/9M4KWgRGNa0" frameborder="0" allowfullscreen></iframe>
 <br/><br/>
 The dependencies of this API are for now:
 <ul>
 <li> <a href="http://opencv.willowgarage.com">Opencv</a>
 </li><li> <a href="http://pointclouds.org"> PCL (Point Cloud Library)</a>
 </li><li> <a href="http://code.google.com/p/libmv/"> The libmv project</a>
 </li><li> <a href="http://eigen.tuxfamily.org/"> The Eigen library (Needed by PCL and LibMV)</a>
 </li><li> <a href="http://www.boost.org/"> The Boost libraries</a>
 </li><li> <a href="http://www.netlib.org/clapack/"> clapack</a>
 </li><li> <a href="http://www.ics.forth.gr/~lourakis/sba/"> lourakis' sba</a>
 </li></ul>

 </div>
 \endhtmlonly
 \latexonly
 I made a little video to show current reconstruction progress. This is not really a structure from motion as the cameras are fully parameterized, but it's a start...\\
 You can see it here: \url{http://www.youtube.com/embed/9M4KWgRGNa0}.\\
 The dependencies of this API are for now:
 \begin{itemize}
 \item Opencv : \url{http://opencv.willowgarage.com}
 \item PCL (Point Cloud Library) : \url{http://pointclouds.org}
 \item The libmv project : \url{http://code.google.com/p/libmv/}
 \item The Eigen library (Needed by PCL and LibMV) : \url{http://eigen.tuxfamily.org/}
 \item The Boost libraries : \url{http://www.boost.org/}
 \item clapack : \url{http://www.netlib.org/clapack/}
 \item lourakis' sba : \url{http://www.ics.forth.gr/~lourakis/sba/}
 \end{itemize}
 \endlatexonly
 */

#endif