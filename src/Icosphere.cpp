#include "Icosphere.h"


Icosphere::Icosphere( size_t subdivisions )
{
    if ( subdivisions < 0 )
	{
		subdivisions = 0;
	}
	else if ( subdivisions > 10 )
	{
		subdivisions = 10;
	}

	verts.reserve( 12 );

	verts.push_back( new Vec3( 0.0, 0.0, 1.0 ) );				//north pole
	verts.push_back( new Vec3( 0.89442, 0.0, 0.44721 ) );
	verts.push_back( new Vec3( 0.27639, -0.85064, 0.44721 ) );
	verts.push_back( new Vec3( -0.7236, -0.52572, 0.44721 ) );
	verts.push_back( new Vec3( -0.7236, 0.52572, 0.44721 ) );
	verts.push_back( new Vec3( 0.27639, 0.85064, 0.44721 ) );	//end of first row
	verts.push_back( new Vec3( 0.7236, -0.52572, -0.44721 ) );
	verts.push_back( new Vec3( -0.27639, -0.85064, -0.44721 ) );
	verts.push_back( new Vec3( -0.89442, 0.0, -0.44721 ) );
	verts.push_back( new Vec3( -0.27639, 0.85064, -0.44721 ) );
	verts.push_back( new Vec3( 0.7236, 0.52572, -0.44721 ) );	//end of second row
	verts.push_back( new Vec3( 0.0, 0.0, -1.0 ) );				//south pole

	vector<shared_ptr<

	tempIcosahedron[0] = Triangle( frmr::Vec3f( 0.0f, 0.0f, 1.0f ), frmr::Vec3f( 0.27639f, 0.85064f, 0.44721f ), frmr::Vec3f( -0.72360f, 0.52572f, 0.44721f ) );
    tempIcosahedron[1] = Triangle( frmr::Vec3f( 0.0f, 0.0f, 1.0f ), frmr::Vec3f( -0.72360f, 0.52572f, 0.44721f ), frmr::Vec3f( -0.72360f, -0.52572f, 0.44721f ) );
    tempIcosahedron[2] = Triangle( frmr::Vec3f( 0.0f, 0.0f, 1.0f ), frmr::Vec3f( -0.72360f, -0.52572f, 0.44721f ), frmr::Vec3f( 0.27639f, -0.85064f, 0.44721f ) );
    tempIcosahedron[3] = Triangle( frmr::Vec3f( 0.0f, 0.0f, 1.0f ), frmr::Vec3f( 0.27639f, -0.85064f, 0.44721f ), frmr::Vec3f( 0.89442f, 0.0f, 0.44721f ) );
    tempIcosahedron[4] = Triangle( frmr::Vec3f( 0.0f, 0.0f, 1.0f ), frmr::Vec3f( 0.89442f,	0.0f, 0.44721f ), frmr::Vec3f( 0.27639f, 0.85064f, 0.44721f ) );
    tempIcosahedron[5] = Triangle( frmr::Vec3f( 0.89442f, 0.0f, 0.44721f ), frmr::Vec3f( 0.72360f, -0.52572f, -0.44721f ), frmr::Vec3f( 0.72360f, 0.52572f, -0.44721f ) );
    tempIcosahedron[6] = Triangle( frmr::Vec3f( 0.89442f, 0.0f, 0.44721f ), frmr::Vec3f( 0.72360f, 0.52572f, -0.44721f ), frmr::Vec3f( 0.27639f, 0.85064f, 0.44721f ) );
    tempIcosahedron[7] = Triangle( frmr::Vec3f( 0.72360f, 0.52572f, -0.44721f ), frmr::Vec3f( -0.27639f, 0.85064f, -0.44721f ), frmr::Vec3f( 0.27639f, 0.85064f, 0.44721f ) );
    tempIcosahedron[8] = Triangle( frmr::Vec3f( 0.27639f, 0.85064f, 0.44721f ), frmr::Vec3f( -0.27639f, 0.85064f, -0.44721f ), frmr::Vec3f( -0.7236f, 0.52572f, 0.44721f ) );
    tempIcosahedron[9] = Triangle( frmr::Vec3f( -0.27639f, 0.85064f, -0.44721f ), frmr::Vec3f( -0.89442f, 0.0f, -0.44721f ), frmr::Vec3f( -0.7236f, 0.52572f, 0.44721f ) );
    tempIcosahedron[10] = Triangle( frmr::Vec3f( -0.7236f, 0.52572f, 0.44721f ),  frmr::Vec3f( -0.89442f, 0.0f, -0.44721f ), frmr::Vec3f( -0.7236f, -0.52572f, 0.44721f ) );
    tempIcosahedron[11] = Triangle( frmr::Vec3f( -0.7236f, -0.52572f, 0.44721f ), frmr::Vec3f( -0.89442f, 0.0f, -0.44721f ), frmr::Vec3f( -0.27639f, -0.85064f, -0.44721f ) );
    tempIcosahedron[12] = Triangle( frmr::Vec3f( -0.7236f, -0.52572f, 0.44721f ), frmr::Vec3f( -0.27639f, -0.85064f, -0.44721f ), frmr::Vec3f( 0.27639f, -0.85064f, 0.44721f ) );
    tempIcosahedron[13] = Triangle( frmr::Vec3f( 0.27639f, -0.85064f, 0.44721f ), frmr::Vec3f( -0.27639f, -0.85064f, -0.44721f ), frmr::Vec3f( 0.7236f, -0.52572f, -0.44721 ) );
    tempIcosahedron[14] = Triangle( frmr::Vec3f( 0.27639f, -0.85064f, 0.44721f ), frmr::Vec3f( 0.7236f, -0.52572f, -0.44721f ), frmr::Vec3f( 0.89442f, 0.0f, 0.44721f ) );
    tempIcosahedron[15] = Triangle( frmr::Vec3f( 0.0f, 0.0f, -1.0f ), frmr::Vec3f( -0.27639f, -0.85064f, -0.44721f ), frmr::Vec3f( -0.89442f, 0.0f, -0.44721f ) );
    tempIcosahedron[16] = Triangle( frmr::Vec3f( 0.0f, 0.0f, -1.0f ), frmr::Vec3f( 0.7236f, -0.52572f, -0.44721f ), frmr::Vec3f( -0.27639f, -0.85064f, -0.44721f ) );
    tempIcosahedron[17] = Triangle( frmr::Vec3f( 0.0f, 0.0f, -1.0f ), frmr::Vec3f( 0.7236f, 0.52572f, -0.44721f ), frmr::Vec3f( 0.7236f, -0.52572f, -0.44721f ) );
    tempIcosahedron[18] = Triangle( frmr::Vec3f( 0.0f, 0.0f, -1.0f ), frmr::Vec3f( -0.27639f, 0.85064f, -0.44721f ), frmr::Vec3f( 0.7236f, 0.52572f, -0.44721f ) );
    tempIcosahedron[19] = Triangle( frmr::Vec3f( 0.0f, 0.0f, -1.0f ), frmr::Vec3f( -0.89442f, 0.0f, -0.44721f ), frmr::Vec3f( -0.27639f, 0.85064f, -0.44721f ) );

}
