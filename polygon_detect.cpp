typedef struct point_t
{
	double x;
	double y;
} point_t;

static bool point_in_polygon(point_t point, point_t polygon[], uint8_t n_vertices)
{
	bool in = false;

	double px = point.x;
	double py = point.y;

	for (uint8_t i = 0, j = n_vertices - 1; i < n_vertices; j = i++)
	{
		double ix = polygon[i].x;
		double iy = polygon[i].y;
		double jx = polygon[j].x;
		double jy = polygon[j].y;

		if( ((iy > py) != (jy > py)) &&
				(px < ((jx - ix) * (py - iy) / (jy - iy)) + ix))
		{
			in = !in;
		}
	}

	return in;
}
