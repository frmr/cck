#ifndef CCK_SIMPLEX_NOISE_H
#define CCK_SIMPLEX_NOISE_H

//TODO: Acknowledge source

namespace cck
{
	class SimplexNoise
	{
	private:
		const double grad3[12][3];
		int perm[512];

	private:
		double Dot( const double* g, const double x, const double y, const double z ) const;

	public:
		double	Noise( const double x, const double y, const double z ) const;
		double	OctaveNoise( const double x, const double y, const double z, const int octaves, const double persistence, double frequency ) const;
		double	ScaledOctaveNoise( const double x, const double y, const double z, const int octaves, const double persistence, const double frequency, const double boundMin, const double boundMax ) const;

	public:
		SimplexNoise( const unsigned int seed );
	};
}


#endif // CCK_SIMPLEX_NOISE_H
