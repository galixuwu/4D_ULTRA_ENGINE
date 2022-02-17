
// > dimensions
// - light coordinates
// draw OR label; coordinates; laser; rotate; fill, radius, colors / connections \ colors1 \ colors2  etc
// / vertices









#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <gtk/gtk.h>

#define MAX_DOTS 80
#define MAX_CONNEC 10
#define MAX_FACES 40
#define MAX_VERTICES 10
#define DEG2RAD M_PI/180

FILE *dorothea;

typedef struct{
  double x;
  double y;
  double z;
  double w;
} vector;

typedef struct{
  int draw_;
  vector v;
  char *info;
  int rotate_;
  int fill_;
  int raio;
  double color1[3];
  int connec[MAX_CONNEC];
  double color2[MAX_CONNEC][3];
  int width;
  int visible_;
  int laser_;
  char label[10];
} extended_vector;

typedef struct{
  int vertices [MAX_VERTICES];
  int num_vert;
  vector edges [MAX_VERTICES];
  vector n;
  int visible_;
  double color[3];
} face_struct;

face_struct face[MAX_FACES];

int emp_max_faces;
int emp_max_dots;
int dimensions;;
  

vector v_intersec, coord_intersec;
double dareah, dareaw;

gboolean flag_vision = TRUE;
int flag_rotation = 0;

extended_vector init[MAX_DOTS];
vector dots[MAX_DOTS];
vector hyper_dots[MAX_DOTS];
vector mid[MAX_DOTS];
extended_vector view[MAX_DOTS];
vector axis [4];
vector pos;
vector direc_vector, old_direc_vector;
vector floating_point;

vector light;

vector default_vision;

vector base1, base2;

double  motion_x = 0., motion_y = 0.;
double ang_vision = 0, ang_vision_v = 0;

double ang1 = 0, ang2 = 0, ang3 = 0, ang4 = 0, ang5 = 0, ang6 = 0, ang7 = 0, ang8 = 0, ang9 = 0;

double size = 1;
double spacing = 3;
double scale = 100;
double move_scale = 0.2;
double persp_scale_4d = 0.2;
double raio = 8;


GtkWidget *scale1, *scale2, *scale3, *scale4, *scale5, *scale6, *scale7, *scale8, *scale9;





                                             //funções das scales

GtkWidget* create_scale (void)
{
  GtkWidget *scale;

  scale = gtk_scale_new_with_range (GTK_ORIENTATION_HORIZONTAL, -270, 270, 1);
  gtk_scale_set_digits (GTK_SCALE(scale), 1);
  gtk_scale_set_value_pos (GTK_SCALE(scale), GTK_POS_TOP);
  gtk_scale_set_draw_value (GTK_SCALE(scale), TRUE);
  gtk_range_set_value (GTK_RANGE(scale), 0);
  //gtk_scale_add_mark (GTK_SCALE(scale), 0, GTK_POS_BOTTOM, "0");

  return scale;
}

void change_scale1 (GtkWidget *scale, gpointer data)
{
  ang1 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale2 (GtkWidget *scale, gpointer data)
{
  ang2 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale3 (GtkWidget *scale, gpointer data)
{
  ang3 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}

void change_scale4 (GtkWidget *scale, gpointer data)
{
  ang4 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale5 (GtkWidget *scale, gpointer data)
{
  ang5 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale6 (GtkWidget *scale, gpointer data)
{
  ang6 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}

void change_scale7 (GtkWidget *scale, gpointer data)
{
  ang7 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale8 (GtkWidget *scale, gpointer data)
{
  ang8 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}


void change_scale9 (GtkWidget *scale, gpointer data)
{
  ang9 = DEG2RAD * gtk_range_get_value (GTK_RANGE(scale));
}

void reset (GtkWidget *w, gpointer data)
{
  gtk_range_set_value (GTK_RANGE(scale1), 0);
  gtk_range_set_value (GTK_RANGE(scale2), 0);
  gtk_range_set_value (GTK_RANGE(scale3), 0);
  gtk_range_set_value (GTK_RANGE(scale4), 0);
  gtk_range_set_value (GTK_RANGE(scale5), 0);
  gtk_range_set_value (GTK_RANGE(scale6), 0);
  gtk_range_set_value (GTK_RANGE(scale7), 0);
  gtk_range_set_value (GTK_RANGE(scale8), 0);
  gtk_range_set_value (GTK_RANGE(scale9), 0);
}

void rodar_func (GtkWidget *w, gpointer data)
{
  flag_rotation = gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(w));
  reset(NULL, NULL);
}





                                         //funções elementares e jogos algébricos (interseções, mudanças de base, etc)

double dot_p (vector v, vector u)
{
  double result;

  result = v.x * u.x + v.y * u.y + v.z * u.z;

  return result;
}

vector cross_p (vector v, vector u)
{
  vector w;
  w.x = v.y * u.z - v.z * u.y;
  w.y = v.z * u.x - v.x * u.z;
  w.z = v.x * u.y - v.y * u.x;

  return w;
}

vector scaler_p (double l, vector v)
{
  vector w;

  w.x = l * v.x;
  w.y = l * v.y;
  w.z = l * v.z;

  return w;
}

vector normalize (vector v)
{
  vector w;

  w = scaler_p (1/sqrt(dot_p(v, v)), v);

  return w;
}
  

vector sum3 (vector v, vector u, vector t)
{
  vector w;
  
  w.x = v.x + u.x + t.x;
  w.y = v.y + u.y + t.y;
  w.z = v.z + u.z + t.z;

  return w;
}

vector diff_vector (vector p, vector q)
{
  vector w;

  w.x = p.x - q.x;
  w.y = p.y - q.y;
  w.z = p.z - q.z;

  return w;
}

vector sum_vector (vector p, vector l)
{
  vector w;

  w.x = p.x + l.x;
  w.y = p.y + l.y;
  w.z = p.z + l.z;

  return w;
}

//função que nao se revelou útil (por agora)
vector find_last_vector (vector n)
{
  vector w;
  double m;

  m = n.y / n.x;

  w.y = 1 / sqrt(m*m + 1);
  w.x = -m * w.y;

  return w;
}
  


vector basis_change (vector b1, vector b2, vector v)
{
  double a, b;
  vector w;
  a = dot_p (v, b2);
  b = dot_p (v, b1);

  w.x = a;
  w.y = b;

  return w;
}

vector intersec (vector l0, vector Q, vector p0, vector n)
//reta que passa por l0 e Q, plano que passa em p0 e é perpendicular a n
{
  double d, m;
  vector l, w;

  l = diff_vector (Q, l0);

  m = dot_p (l, n);

  d = dot_p (diff_vector (p0, l0), n) / m;

  w = sum_vector (l0, scaler_p (d, l));
	  
  return w;
}





                                                         //funções interação com o utlizador

gboolean keypress_func (GtkWidget *widget, GdkEventKey *event, gpointer data)
{
  if (event->keyval == GDK_KEY_w)
    pos = sum_vector (pos, scaler_p (move_scale, old_direc_vector));

  if (event->keyval == GDK_KEY_s)
    pos = sum_vector (pos, scaler_p (-move_scale, old_direc_vector));

  if (event->keyval == GDK_KEY_a)
    pos = sum_vector (pos, scaler_p (move_scale, base2));

  if (event->keyval == GDK_KEY_d)
    pos = sum_vector (pos, scaler_p (-move_scale, base2));

  if (event->keyval == GDK_KEY_space)
    pos.z += move_scale;

  if (event->keyval == GDK_KEY_f)
    pos.z -= move_scale;
  
  if (event->keyval == GDK_KEY_Up)
    spacing += 0.01;

  if (event->keyval == GDK_KEY_Down)
    spacing -= 0.01;

  if (event->keyval == GDK_KEY_l)
    {
      if (flag_vision)
	{
	  default_vision.x += motion_x;
	  default_vision.y += motion_y;
	  flag_vision = FALSE;
	}
      else
	{
	  default_vision.x -= motion_x;
	  default_vision.y -= motion_y;
	  flag_vision = TRUE;
	}
    }
      
  if (event->keyval == GDK_KEY_Right)
    persp_scale_4d += 0.01;

  if (event->keyval == GDK_KEY_Left)
    persp_scale_4d -= 0.01;

  return TRUE;
}


gboolean cb_motion_notify (GtkWidget *widget, GdkEventExpose *event, GtkWidget *win)
{

  if (event->type == GDK_MOTION_NOTIFY)
    {
      motion_x = ((GdkEventMotion *)event)->x;
      motion_y = ((GdkEventMotion *)event)->y;

      if (flag_vision)
	{
	  ang_vision = (dareaw/2 - motion_x - default_vision.x) * 2 * M_PI / dareaw;
	  ang_vision_v = (dareah/2 - motion_y - default_vision.y) * 2 * M_PI / dareah;

	  old_direc_vector.x = cos (ang_vision);
	  old_direc_vector.y = sin (ang_vision);
	  old_direc_vector.z = 0;
	  
	  direc_vector.x = cos (ang_vision) * cos (ang_vision_v);
	  direc_vector.y = sin (ang_vision) * cos (ang_vision_v);
	  direc_vector.z = sin (ang_vision_v);
	  
	  base2.x = -sin (ang_vision);
	  base2.y = cos (ang_vision);
	  base2.z = 0;

	  base1.x = - cos(ang_vision) * sin (ang_vision_v);
	  base1.y = -sin(ang_vision) * sin (ang_vision_v);
	  base1.z = cos (ang_vision_v);
	}
    }

  return FALSE;
}





                                                  //funções das equações da rotação, equações da perspetiva artificial (4d->3d) e jogo da perspetiva natural

//a ideia da função matriz nao funcionou

void equations(void)
{
  int i1;


  for (i1 = 0; i1 < MAX_DOTS; i1++)
    {
      hyper_dots[i1] = init[i1].v;

      if (init[i1].rotate_ && flag_rotation)
	{
	  if (ang1 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].x = cos(ang1) * mid[i1].x - sin(ang1) * mid[i1].y;
	      hyper_dots[i1].y = sin(ang1) * mid[i1].x + cos(ang1) * mid[i1].y;
	    }
	  if (ang2 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].x = cos(ang2) * mid[i1].x - sin(ang2) * mid[i1].z;
	      hyper_dots[i1].z = sin(ang2) * mid[i1].x + cos(ang2) * mid[i1].z;
	    }
	  if (ang3 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].x = cos(ang3) * mid[i1].x - sin(ang3) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang3) * mid[i1].x + cos(ang3) * mid[i1].w;
	    }
	  if (ang4 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].y = cos(ang4) * mid[i1].y - sin(ang4) * mid[i1].z;
	      hyper_dots[i1].z = sin(ang4) * mid[i1].y + cos(ang4) * mid[i1].z;
	    }
	  if (ang5 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].y = cos(ang5) * mid[i1].y - sin(ang5) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang5) * mid[i1].y + cos(ang5) * mid[i1].w;
	    }
	  if (ang6 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].z = cos(ang6) * mid[i1].z - sin(ang6) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang6) * mid[i1].z + cos(ang6) * mid[i1].w;
	    }
	  if (ang7 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].x = cos(ang7) * mid[i1].x - sin(ang7) * mid[i1].y;
	      hyper_dots[i1].y = sin(ang7) * mid[i1].x + cos(ang7) * mid[i1].y;
	      hyper_dots[i1].z = cos(ang7) * mid[i1].z - sin(ang7) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang7) * mid[i1].z + cos(ang7) * mid[i1].w;
	    }
	  if (ang8 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].x = cos(ang8) * mid[i1].x - sin(ang8) * mid[i1].z;
	      hyper_dots[i1].z = sin(ang8) * mid[i1].x + cos(ang8) * mid[i1].z;
	      hyper_dots[i1].y = cos(ang8) * mid[i1].y - sin(ang8) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang8) * mid[i1].y + cos(ang8) * mid[i1].w;
	    }
	  if (ang9 != 0)
	    {
	      mid[i1] = hyper_dots[i1];
	      hyper_dots[i1].y = cos(ang9) * mid[i1].y - sin(ang9) * mid[i1].z;
	      hyper_dots[i1].z = sin(ang9) * mid[i1].y + cos(ang9) * mid[i1].z;
	      hyper_dots[i1].x = cos(ang9) * mid[i1].x - sin(ang9) * mid[i1].w;
	      hyper_dots[i1].w = sin(ang9) * mid[i1].x + cos(ang9) * mid[i1].w;
	    }
	}
	  
    }
}


void persp_equations_4d_to_3d (void)
{
  int i1;

  for (i1 = 0; i1 < MAX_DOTS; i1++)
    {
      dots[i1] = scaler_p (pow (M_E, (hyper_dots[i1].w) * persp_scale_4d), hyper_dots[i1]);
    }

}

void persp_3d (void)
{

  int i1;

  for (i1 = 0; i1 < MAX_DOTS; i1++)
    {
      
      if (dot_p (direc_vector, diff_vector (pos, dots[i1])) < 0)
	{
	  v_intersec = intersec (dots[i1], pos, floating_point, direc_vector);
	  
	  view[i1] = init[i1];
	  
	  view[i1].v = basis_change (base1, base2, diff_vector (v_intersec, floating_point));
	  view[i1].visible_ = 1;

	  view[i1].v.x = dareaw/2 - scale * view[i1].v.x;
	  view[i1].v.y = dareah/2 - scale * view[i1].v.y;
	  view[i1].v.z = 0;
	  view[i1].v.w = 0;
	  
	}
      else
	{
	  view[i1].visible_ = 0;
	}
    }
}



//função para testar a visibilidade de cada ponto, checakando se a sua projeção está dentro de alguma face (testa todas as faces contrárias para cada ponto)


void visibilities (void)
{
  int i1, i2, i3;
  int m0, m1, m2;
  int troca[MAX_FACES];
  int vamos_ver;
  double pls;

  for (i2 = 0; i2 < emp_max_faces; i2++)
    {
      for (i3 = 0; i3 < face[i2].num_vert; i3++)
	{
	  face[i2].edges[i3] = diff_vector (view[face[i2].vertices[(i3+1)%(face[i2].num_vert)]].v, view[face[i2].vertices[i3]].v);

	  m0 = face[i2].vertices[0];
	  m1 = face[i2].vertices[1];
	  m2 = face[i2].vertices[2];
	  
	  face[i2].n = normalize(cross_p(diff_vector(dots[m1], dots[m0]), diff_vector(dots[m2], dots[m1])));
	}
      
      if (cross_p(face[i2].edges[0], face[i2].edges[1]).z <= 0)
	troca[i2] = -1;
      else
	troca[i2] = 1;
    }

  for (i1 = 0; i1 < MAX_DOTS; i1++)
    {
      if (view[i1].visible_ && view[i1].draw_ && !view[i1].laser_)
	{
	  for (i2 = 0; i2 < emp_max_faces; i2++)
	    {
	      if (dot_p (face[i2].n, diff_vector (pos, dots[i1])) > 0)
		{
		  vamos_ver = 0;
		  
		  for (i3 = 0; i3 < face[i2].num_vert; i3++)
		    {
		      pls = cross_p (face[i2].edges[i3], diff_vector (view[i1].v, view[face[i2].vertices[i3]].v)).z;
		      if (troca[i2] * pls <= 0 && view[i1].visible_)
			{
			  view[i1].visible_ = 1;
			  vamos_ver++;
			  break;
			}
		    }
		  
		  if (vamos_ver == 0)
		    view[i1].visible_ = 0;
		}
	    }
	}
    }

  int olivia;

  for (i2 = 0; i2 < emp_max_faces; i2++)
    {
      olivia = 0;

      face[i2].color[0] = .9;
      face[i2].color[1] = .6;
      face[i2].color[2] = .8;
      
      for (i3 = 0; i3 < face[i2].num_vert; i3++)
	{
	  if (!view[face[i2].vertices[i3]].visible_)
	    {
	      face[i2].visible_ = 0;
	      olivia++;
	    }
	}
      if (olivia == 0)
	{
	  face[i2].visible_ = 1;
	}
    }
}
		      
                                                                  //funções de desenho
void full_draw (GtkWidget *darea, cairo_t *cr, gpointer data)
{
  
  double c1, c2, c3;

  int r, w, m;

  int i1, j1, i2, i3;

  double ratio;

  cairo_set_line_width (cr, 1);

  for (i2 = 0; i2 < emp_max_faces; i2++)
    {
      if (face[i2].visible_)
	{
	  c1 = face[i2].color[0];
	  c2 = face[i2].color[1];
	  c3 = face[i2].color[2];

	  ratio = (1 - dot_p (light, face[i2].n))/2;
	  
	  cairo_set_source_rgb (cr, ratio * c1 , ratio * c2, ratio * c3);

	  cairo_move_to (cr, view[face[i2].vertices[0]].v.x, view[face[i2].vertices[0]].v.y);

	  for (i3 = 0; i3 < face[i2].num_vert; i3++)
	    {
	      cairo_line_to (cr, view[face[i2].vertices[(i3+1)%face[i2].num_vert]].v.x, view[face[i2].vertices[(i3+1)%face[i2].num_vert]].v.y);
	    }
	  cairo_fill (cr);
	}
    }
  
  cairo_stroke (cr);

  for (i1 = 0; i1 < MAX_DOTS; i1++)
    {
      if (view[i1].draw_ && view[i1].visible_)
	{	  
	  w = view[i1].width;
	  
	  cairo_set_line_width (cr, w);
	  
	  for (j1 = 0; j1 < MAX_CONNEC; j1++)
	    {
	      c1 = view[i1].color2[j1][0];
	      c2 = view[i1].color2[j1][1];
	      c3 = view[i1].color2[j1][2];

	      cairo_set_source_rgb (cr, c1, c2, c3);
	      
	      m = view[i1].connec[j1];
	      
	      if (m == 0)
		break;
	      else if (view[m].draw_ && view[m].visible_)
		{
		  cairo_move_to (cr, view[i1].v.x, view[i1].v.y);
		  cairo_line_to (cr, view[m].v.x, view[m].v.y);
		}

	      cairo_stroke (cr);
	    }
      
	  
	  if (view[i1].fill_)
	    {
	      c1 = view[i1].color1[0];
	      c2 = view[i1].color1[1];
	      c3 = view[i1].color1[2];
	      
	      r = view[i1].raio;
	      
	      cairo_set_line_width (cr, 1.);
	      cairo_set_source_rgb (cr, c1, c2, c3);
	      
	      cairo_arc (cr, view[i1].v.x, view[i1].v.y, r, 0, 2*M_PI);
	      
	      cairo_fill (cr);
	      cairo_stroke (cr);
	      
	      cairo_move_to (cr, view[i1].v.x + 10, view[i1].v.y + 10);
	      cairo_show_text (cr, view[i1].label);
	      cairo_stroke (cr);
	    }
	}

    }
}


gboolean cb_draw (GtkWidget *darea, cairo_t *cr, gpointer data)
{
  GtkAllocation alloc1;
  gtk_widget_get_allocation (darea, &alloc1);

  dareah = alloc1.height;
  dareaw = alloc1.width;

  floating_point = sum_vector (pos, scaler_p(spacing, direc_vector));

  equations();
  persp_equations_4d_to_3d();
  persp_3d();
  visibilities();

  full_draw (darea, cr, NULL);

  return FALSE;
}

gboolean time_handler (GtkWidget *widget)
{
  if (!GTK_IS_WIDGET(widget) || (!gtk_widget_get_window (widget)))
    return FALSE;

  gtk_widget_queue_draw (widget);
  return TRUE;
}



                                                          //valores iniciais dos vetores

void vectors_init (void)
{

  pos.x = -8;
  pos.y = 0;
  pos.z = 0;

  default_vision.x = 0;
  default_vision.y = 0;
  default_vision.z = 0;

  char *smid;
  smid = (char*) malloc (99 * sizeof (char));
	  
  int i1 = 0, teste1, teste2, teste3, imid, i2, teste4;
  int i3 = 0, teste5, teste6;
  int m0, m1, m2;

  while (fgets(smid, 99, dorothea) != NULL)
    {
      //printf ("oi - %d - %c\n", i1, smid[0]);

      if (smid[0] == 62)   //62 == >
	{
	  dimensions = smid[2] - 48;
	}
      else if (smid[0] == 45)   //45 == -
	{
	  sscanf(smid, "- %lf, %lf, %lf", &light.x, &light.y, &light.z);
	  light = normalize (light);
	}
      else if (smid[0] == 47)
	{
	  teste5 = sscanf(smid, "/ %d %d %d %d %d %d", &face[i3].vertices[0], &face[i3].vertices[1], &face[i3].vertices[2], &face[i3].vertices[3], &face[i3].vertices[4], &face[i3].vertices[5]);

	  //printf ("%d\n", teste5);

	  face[i3].num_vert = teste5;
	  

	  if (teste5 < 3)
	    break;
	  
	  smid = strchr (smid, 47);
	  
	  teste6 = sscanf(smid, "/ %lf %lf %lf", &face[i3].color[0], &face[i3].color[1], &face[i3].color[2]);
	  
	  if (teste6 == 0)
	    {
	      face[i3].color[0] = 0.3;
	      face[i3].color[1] = 0.3;
	      face[i3].color[2] = 0.3;
	    }
	  i3++; 
	}
      else
	{
	  init[i1].info = (char*) malloc (99 * sizeof (char));
	  init[i1].info = smid;


	  if (smid[0] >= 48 && smid[0] <= 57)
	    {
	      sscanf(smid, "%d", &init[i1].draw_);
	      sprintf(init[i1].label, "%d", init[i1].draw_);
	    }
	  else
	    {
	      init[i1].draw_ = 1;
	      init[i1].label[0] = smid[0];
	    }

	  

	  smid = strchr (&smid[0], 59);

	  if (dimensions == 4)
	    {
	      teste1 = sscanf (smid, "; %lf, %lf, %lf, %lf; %d; %d; %d, %d, %lf %lf %lf / %d", &init[i1].v.x, &init[i1].v.y, &init[i1].v.z, &init[i1].v.w, &init[i1].laser_, &init[i1].rotate_, &init[i1].fill_, &init[i1].raio, &init[i1].color1[0], &init[i1].color1[1], &init[i1].color1[2], &imid);
	    }
	  else if (dimensions == 3)
	    {
	      teste1 = sscanf (smid, "; %lf, %lf, %lf; %d; %d; %d, %d, %lf %lf %lf / %d", &init[i1].v.x, &init[i1].v.y, &init[i1].v.z, &init[i1].laser_, &init[i1].rotate_, &init[i1].fill_, &init[i1].raio, &init[i1].color1[0], &init[i1].color1[1], &init[i1].color1[2], &imid);
	      init[i1].v.w = 0;
	    }
	  
	  //printf ("%d\n", teste1);
	  
	  if (teste1 == 8 + dimensions)
	    {
	      smid = strchr (smid, 47);   //47 = /
	      
	      teste2 = sscanf (smid, "/ %d %d %d %d %d %d %d %d %d %d", &init[i1].connec[0], &init[i1].connec[1], &init[i1].connec[2], &init[i1].connec[3], &init[i1].connec[4], &init[i1].connec[5], &init[i1].connec[6], &init[i1].connec[7], &init[i1].connec[8], &init[i1].connec[9]);
	    }
	  else
	    teste2 = 0;
	  
	  //printf ("%d\n", teste2);
	  
	  if (teste2)
	    {
	      for (i2 = 0; i2 < teste2; i2++)
		{
		  smid = strchr (&smid[1], 92);    //92 = \ //
		  
		  teste3 = sscanf (smid, "\\ %lf %lf %lf", &init[i1].color2[i2][0], &init[i1].color2[i2][1], &init[i1].color2[i2][2]);
		  
		  //printf("%d\n", teste3);
		}
	      
	      //printf ("%lf\n", init[i1].color2[1][0]);
	      
	      smid = strchr (smid, 59);   //59 = ;
	      
	      teste4 = sscanf (smid, "; %d", &init[i1].width);
	      
	      //printf("%d\n", teste4);
	    }
	  
	  
	  //printf ("\n");
	  
	  
	  i1++;
	}
    }


  emp_max_faces = i3;
  emp_max_dots = i1;
}





                                                 //função main

int main(int argc, char **argv)
{
  gtk_init (&argc, &argv);
  
  dorothea = fopen (argv[1], "rt");
  
  if (dorothea == NULL)
    {
      printf ("ERR\n");
      return -1;
    }
  
  
  vectors_init ();

  GtkWidget *window, *caixa1, *caixa2, *frame, *darea;
  GtkWidget *label1, *label2, *label3, *label4, *label5, *label6, *label7, *label8, *label9;
  GtkWidget *botao_reset, *botao_rodar;

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_default_size (GTK_WINDOW(window), 1500, 900);
  gtk_window_set_title (GTK_WINDOW(window), "stream gabo on spotify");
  gtk_window_set_position (GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  g_signal_connect (G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

  caixa1 = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 0);
  gtk_container_add (GTK_CONTAINER(window), caixa1);

  caixa2 = gtk_box_new (GTK_ORIENTATION_VERTICAL, 0);
  gtk_box_pack_end (GTK_BOX(caixa1), caixa2, TRUE, TRUE, 3);

  label1 = gtk_label_new ("xy");
  label2 = gtk_label_new ("xz");
  label3 = gtk_label_new ("xw");
  label4 = gtk_label_new ("yz");
  label5 = gtk_label_new ("yw");
  label6 = gtk_label_new ("zw");

  scale1 = create_scale();
  scale2 = create_scale();
  scale3 = create_scale();
  scale4 = create_scale();
  scale5 = create_scale();
  scale6 = create_scale();
  
  gtk_box_pack_start (GTK_BOX(caixa2), label1, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale1, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label2, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale2, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label3, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale3, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label4, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale4, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label5, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale5, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label6, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale6, FALSE, FALSE, 3);
  
  g_signal_connect (G_OBJECT(scale1), "value-changed", G_CALLBACK(change_scale1), NULL);
  g_signal_connect (G_OBJECT(scale2), "value-changed", G_CALLBACK(change_scale2), NULL);
  g_signal_connect (G_OBJECT(scale3), "value-changed", G_CALLBACK(change_scale3), NULL);
  g_signal_connect (G_OBJECT(scale4), "value-changed", G_CALLBACK(change_scale4), NULL);
  g_signal_connect (G_OBJECT(scale5), "value-changed", G_CALLBACK(change_scale5), NULL);
  g_signal_connect (G_OBJECT(scale6), "value-changed", G_CALLBACK(change_scale6), NULL);

  //double rotations, clifford

  label7 = gtk_label_new ("xy + zw");
  label8 = gtk_label_new ("xz + yw");
  label9 = gtk_label_new ("yz + xw");

  scale7 = create_scale();
  scale8 = create_scale();
  scale9 = create_scale();

  gtk_box_pack_start (GTK_BOX(caixa2), label7, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale7, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label8, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale8, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), label9, FALSE, FALSE, 3);
  gtk_box_pack_start (GTK_BOX(caixa2), scale9, FALSE, FALSE, 3);

  g_signal_connect (G_OBJECT(scale7), "value-changed", G_CALLBACK(change_scale7), NULL);
  g_signal_connect (G_OBJECT(scale8), "value-changed", G_CALLBACK(change_scale8), NULL);
  g_signal_connect (G_OBJECT(scale9), "value-changed", G_CALLBACK(change_scale9), NULL);

  botao_reset = gtk_button_new_with_label ("Reset");
  gtk_box_pack_start (GTK_BOX(caixa2), botao_reset, FALSE, FALSE, 3);
  g_signal_connect (G_OBJECT(botao_reset), "clicked", G_CALLBACK(reset), NULL);

  botao_rodar = gtk_toggle_button_new_with_label ("Rotations");
  gtk_box_pack_start (GTK_BOX(caixa2), botao_rodar, FALSE, FALSE, 3);
  g_signal_connect (G_OBJECT(botao_rodar), "toggled", G_CALLBACK(rodar_func), NULL);

  frame = gtk_frame_new ("gabriel seixas na lupa");
  //gtk_frame_set_label_align (GTK_FRAME(frame), 0.5, 0.5);
  gtk_box_pack_start (GTK_BOX(caixa1), frame, TRUE, TRUE, 3);

  darea = gtk_drawing_area_new ();
  gtk_widget_set_size_request(darea, 1000, 900);
  gtk_container_add (GTK_CONTAINER(frame), darea);

  g_signal_connect (G_OBJECT(darea), "draw", G_CALLBACK(cb_draw), NULL);
  
  g_timeout_add (10, (GSourceFunc) time_handler, darea);

  gtk_widget_set_events (window, GDK_POINTER_MOTION_MASK | GDK_KEY_PRESS_MASK);

  g_signal_connect (G_OBJECT(window), "key_press_event", G_CALLBACK(keypress_func), NULL);
  
  g_signal_connect (window, "motion-notify-event", G_CALLBACK (cb_motion_notify), NULL);

  gtk_widget_show_all (window);
  gtk_main();

  return 12;

}






// TA FEITOOOOOOOOOOOOO
// obg zé amigo
