#ifndef CCK_SIMPLEX_NOISE_H
#define CCK_SIMPLEX_NOISE_H

namespace cck
{
	class SimplexNoise
	{
	private:
		int perm[512];

		const double grad3[12][3] = {	{ 1.0, 1.0, 0.0 }, { -1.0, 1.0, 0.0 }, { 1.0, -1.0, 0.0},	{ -1.0, -1.0, 0.0 },
										{ 1.0, 0.0, 1.0 }, { -1.0, 0.0, 1.0 }, { 1.0, 0.0, -1.0 },	{ -1.0, 0.0, -1.0 },
										{ 0.0, 1.0, 1.0 }, { 0.0, -1.0, 1.0 }, { 0.0, 1.0, -1.0 },	{ 0.0, -1.0, -1.0 }	};

	private:
		double Dot( const double g[], const double x, const double y, const double z ) const; //TODO: Specify array size

	public:
		double	Noise( const double x, const double y, const double z ) const;
		double	OctaveNoise( const double x, const double y, const double z, const int octaves, const double persistence, double frequency ) const;
		double	ScaledOctaveNoise( const double x, const double y, const double z, const int octaves, const double persistence, const double frequency, const double loBound, const double hiBound ) const;

	public:
		SimplexNoise( const unsigned int seed );
	};
}


#endif // SIMPLEX_NOISE_GENERATOR_H
