/*
CAVINA MATTIA
MATRICOLA:0001113736
GRUPPO A
email : mattia.cavina2@studio.unibo.it
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#define INF 1874919423
#define GIALLO 1
#define BIANCO 2

/*Struttura degli archi del grafo*/
typedef struct Edge {
    int src;            /* nodo sorgente        */
    int dst;            /* nodo destinazione    */
    double weight;      /* peso dell'arco       */
    struct Edge* next;

    int xsrc;           /*posizione delle nodo di provenienza*/
    int ysrc;
    int livsrc;

    int xdst;           /*posizione del nodo di destinazione*/
    int ydst; 
    int livdst;

    int Colore;         /*Valori per gestire un algoritmo di cammino minimo*/
    struct Edge* padre; /*Puntatore al padre per delineare i cammini minimi*/
    int  key;           /*peso dell arco*/

} Edge;


typedef struct Mappagrafo
{
    int livello;        /*altezza in una cella*/
    int posizione;      /*posizione incremenatle nella mappa*/

    int x;              /*colonne*/
    int y;              /*righe*/
}Mappagrafo;

typedef struct Graph {
    int n;              /* numero di nodi               */
    int m;              /* numero di archi              */
    Edge** edges;       /* array di liste di adiacenza  */
    int* in_deg;       /* grado entrante dei nodi      */
    int* out_deg;       /* grado uscente dei nodi       */

    int ccell;          /*costo spostamento orizontale*/
    int cheight;        /*costo verticale orizontale*/
    
    int righe ;         /*numero righe matrice*/
    int colonne;        /*numero colonne matrice*/

    Mappagrafo** mappa;/*puntatore di puntatori alla mappa scansionata*/

} Graph;

/*Struttura per gestire i nodi analizzati del grafo*/
typedef struct Nodoanallizzato/*Struttura di appoggio per gestire i nodi che analizzo nell algortimo di dijkstra*/
{
    int num;            /*numero del nodo*/
    int key;            /*peso del nodo*/
    int colore;         /*colore per identificare se analizzato o meno*/

}Nodoanallizzato;


/*Costruzione del grafo*/

/*Funzione che mi permette di costruire il grafo */
Graph* Costruiscigrafo(FILE* fp);

/*Scansiona il file dato e crea una matrice corrispondneti ai punti della mappa con i corrispettivi dati , esporta il punattore alla mappa*/
Mappagrafo** creamappa(FILE* fp, int righe, int colonne);

/*Funzione che mi permette di creare archi da un punto dell arco ai punti vicini calcolando i pesi degli archi corrispondenti*/
void edgebuilding(Graph* nuovo, Mappagrafo** mappa, int i, int k);

/*Funzione che calcola il peso dell arco da provenienza a destinazione*/
double calcolapeso(int liviniz, int livfin, int ccell, int cheight);

/* Crea un nuovo grafo con `n` nodi. Il numero di nodi deve essere
   strettamente positivo. */
Graph* graph_create(int n);

/*Funzione per aggiungere un nuovo arco*/
static Edge* new_edge(Mappagrafo src, Mappagrafo dst, double weight, Edge* next);

/* Inserisce l'arco (src, dst, weight) nel grafo */
static int graph_adj_insert(Graph* g, Mappagrafo src, Mappagrafo dst, double weight);

/* Aggiunge un nuovo arco (src, dst) con peso "weight".*/
void graph_add_edge(Graph* g, Mappagrafo src, Mappagrafo dst, double weight);

/* Restituisce un puntatore al primo arco della lista di adiacenza
   associata al nodo `v` (`NULL` se la lista è vuota) */
Edge* graph_adj(const Graph* g, int v);

/* Stampa il grafo a video */
void graph_print(const Graph* g);

/* Restituisce il numero di nodi del grafo */
int graph_n_nodes(const Graph* g);

/* Restituisce il grado uscente del nodo `v`. */
int graph_out_degree(const Graph* g, int v);

/*Elimino tutta la memoria allocata del mio grafo*/
void graph_destroy(Graph* g);

/*Dijkstra*/

/*Funzione principale per gestire l'algoritmo di dijkstra , immettendo solo il grafo di partenza.*/
void dijkstra(Graph* g);

/*rilassa gli archi dei nodi analizzati*/
void relax(Edge* arcosrc, Edge** archi, int dst);

/*inizializzo tutti i nodi e archi del mio grafo*/
void inizkey(Edge** archi, int nodo);

/*Creo un vettore ordinato per gestire i nodi del mio grafo*/
Nodoanallizzato* buildvet(Graph* g);

/*Aggiorno il key del nodo analizzato*/
void aggvet(Nodoanallizzato* lista, int num, int key);

/*Estraggo  il nodo con key minore , ovvero quello con un peso del persorso
ritorno il numero del nodo analizzato*/
int prevnum(Nodoanallizzato* lista, int nodi);

/*Funzione che dato un punto di arrivo mi trovi la strada più breve per la partenza*/
double percorso(Edge* archi, int nodo, int ccel);


int main(int argc, char* argv[])
{
    Graph* Grafo;
    FILE* fp = stdin;

    if (argc != 2) {
        fprintf(stderr, "Invocare il programma con: %s \n", argv[0]);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "-") != 0) {
        fp = fopen(argv[1], "r");
        if (fp == NULL) {
            fprintf(stderr, "Can not open %s\n", argv[1]);
            return EXIT_FAILURE;
        }
    }

    Grafo = Costruiscigrafo(fp);

    dijkstra(Grafo);

    graph_destroy(Grafo);/*libero la memoria del mio grafo*/


    if (fp != stdin) fclose(fp);

    return EXIT_SUCCESS;
}
/*
int main(void)
{
    Graph* Grafo;
    FILE* fp;

    fp = fopen("test1.in", "r");
    assert(fp != NULL);

    Grafo = Costruiscigrafo(fp);

    fclose(fp);

    dijkstra(Grafo);

    graph_destroy(Grafo);

    return EXIT_SUCCESS;
}
*/
/*Funzione che mi permette di costruire il grafo */
Graph* Costruiscigrafo(FILE* fp)
{
    Graph* nuovo;

    int ccell, cheight, righe, colonne;

    int i, k; /*indice*/

    fscanf(fp, "%d", &ccell);/*leggo i valori dal file e salvo i dati*/
    fscanf(fp, "%d", &cheight);
    fscanf(fp, "%d", &righe);
    fscanf(fp, "%d", &colonne);
    assert(colonne != 0);
    
    
    nuovo = graph_create(righe * colonne);/*creo un il grafo*/
    assert(nuovo != NULL);

    nuovo->ccell = ccell;/*Salvo nel grafo i dati necessari*/
    nuovo->cheight = cheight;
    nuovo->colonne = colonne;
    nuovo->righe = righe;
    

    nuovo->mappa = creamappa(fp , nuovo->righe , nuovo->colonne);/*creo una matrice e l'appoggio nel puntatore del mio grado*/
    assert(nuovo->mappa != NULL);


    for (i = 0 ; i < nuovo->righe ; i++)/*inserisco gli archi per ogni nodo della mia mappa*/
    {
        for (k = 0 ; k < nuovo->colonne ; k++)
        {           
            edgebuilding(nuovo, nuovo->mappa, i, k);
        }
    };

    assert(nuovo->edges != NULL);

    return nuovo;

}

/*Scansiona il file dato e crea una matrice corrispondneti ai punti della mappa con i corrispettivi dati , esporta il punattore alla mappa*/
Mappagrafo **creamappa(FILE* fp , int righe, int colonne)
{
    int i , k , j;/*indici*/
    
    Mappagrafo** mappa;

    mappa = (Mappagrafo**)malloc(sizeof(Mappagrafo) * righe);/*alloco dinamicamente la dimensione delle righe*/
    
    assert(mappa != NULL);

    for (i = 0 ; i < righe ; i++)
    {
        mappa[i] = (Mappagrafo*)malloc(sizeof(Mappagrafo) * colonne);/*alloco dinamicamente la dimensione delle colonne*/
        
        assert(mappa[i] != NULL);
    }

    

    for (i = 0 , j = 0 ; i < righe ; i++)
    {

        for (k = 0 ; k < colonne ; k++ , j++)
        {
            fscanf(fp, "%d" , &mappa[i][k].livello); /*dal file scansiono l'altezza della cella e la sua posizione*/
            mappa[i][k].posizione = j;
            mappa[i][k].x = k;
            mappa[i][k].y = i;
  
        }
    }
    
    assert(mappa[0][0].posizione == 0);

    return mappa;

}


/*Funzione che mi permette di creare archi da un punto dell arco ai punti vicini calcolando i pesi degli archi corrispondenti*/
void edgebuilding(Graph* nuovo , Mappagrafo** mappa , int i, int k)
{
    double weight = 0;

    if ((i + 1) < nuovo->righe)
    {
        weight = calcolapeso(mappa[i][k].livello, mappa[i + 1][k].livello, nuovo->ccell, nuovo->cheight);
        graph_add_edge(nuovo, mappa[i][k], mappa[i + 1][k], weight );
        graph_add_edge(nuovo, mappa[i+1][k], mappa[i][k], weight);

    }

    if ((k + 1) < nuovo->colonne)
    {
        weight = calcolapeso(mappa[i][k].livello, mappa[i][k + 1].livello, nuovo->ccell, nuovo->cheight);
        graph_add_edge(nuovo, mappa[i][k], mappa[i][k + 1], weight); 
        graph_add_edge(nuovo, mappa[i][k+1], mappa[i][k], weight);

    }
    
}

/*Funzione che calcola il peso dell arco da provenienza a destinazione*/
double calcolapeso(int liviniz, int livfin, int ccell, int cheight)
{
    double result;
    int dislivello;
   
    dislivello = liviniz - livfin;

    result = cheight * (dislivello * dislivello) + ccell;

    return result;
}


/* Crea un nuovo grafo con `n` nodi. Il numero di nodi deve essere
   strettamente positivo. */
Graph* graph_create(int n)
{
    int i;
    Graph* g = (Graph*)malloc(sizeof(Graph));
    assert(g != NULL);
    assert(n > 0);

    g->n = n;
    g->m = 0;
    g->edges = (Edge**)malloc(n * sizeof(Edge*));
    assert(g->edges != NULL);
    g->in_deg = (int*)malloc(n * sizeof(*(g->in_deg)));
    assert(g->in_deg != NULL);
    g->out_deg = (int*)malloc(n * sizeof(*(g->out_deg)));
    assert(g->out_deg != NULL);
    for (i = 0; i < n; i++) {
        g->edges[i] = NULL;
        g->in_deg[i] = g->out_deg[i] = 0;
    }

    assert(g->n != 0);
    return g;
}

/*Funzione per aggiungere un nuovo arco*/
static Edge* new_edge(Mappagrafo src, Mappagrafo dst, double weight, Edge* next)
{
    Edge* edge = (Edge*)malloc(sizeof(Edge));
    assert(edge != NULL);

    edge->src = src.posizione;
    edge->dst = dst.posizione;
    edge->weight = weight;
    edge->next = next;

    edge->xsrc = src.x;
    edge->ysrc = src.y;
    edge->livsrc = src.livello;

    edge->xdst = dst.x;
    edge->ydst = dst.y;
    edge->livdst = dst.livello;

    return edge;
}

/* Inserisce l'arco (src, dst, weight) nel grafo */
static int graph_adj_insert(Graph* g, Mappagrafo src, Mappagrafo dst, double weight)
{

    g->edges[src.posizione] = new_edge(src, dst, weight, g->edges[src.posizione]);
    g->in_deg[dst.posizione]++;
    g->out_deg[src.posizione]++;
    return 0;
}

/* Aggiunge un nuovo arco (src, dst) con peso "weight".*/
void graph_add_edge(Graph* g, Mappagrafo src, Mappagrafo dst, double weight  )
{
    int status = 0;

    assert(g != NULL);

    assert((src.posizione >= 0) && (src.posizione < graph_n_nodes(g)));
    assert((dst.posizione >= 0) && (dst.posizione < graph_n_nodes(g)));

    status = graph_adj_insert(g, src, dst, weight);

    if (status == 0)
        g->m++;
    else
        fprintf(stderr, "Ho ignorato l'arco duplicato (%d,%d)\n", src.posizione, dst.posizione);
}

/* Restituisce un puntatore al primo arco della lista di adiacenza
   associata al nodo `v` (`NULL` se la lista è vuota) */
Edge* graph_adj(const Graph* g, int v)
{
    assert(g != NULL);
    assert((v >= 0) && (v < graph_n_nodes(g)));

    return g->edges[v];
}

/* Stampa il grafo a video */
void graph_print(const Graph* g)
{
    int i;

    assert(g != NULL);

    for (i = 0; i < g->n; i++) {
        const Edge* e;
        int out_deg = 0; /* ne approfittiamo per controllare la
                            correttezza dei gradi uscenti */
        printf("[%2d] -> ", i);
        for (e = graph_adj(g, i); e != NULL; e = e->next) {
            printf("(%d, %d, %.2f ) -> ", e->src, e->dst, e->weight);
            
            out_deg++;
        }
        assert(out_deg == graph_out_degree(g, i));
        printf("NULL\n");       
    }
}

/* Restituisce il numero di nodi del grafo */
int graph_n_nodes(const Graph* g)
{
    assert(g != NULL);

    return g->n;
}

/* Restituisce il grado uscente del nodo `v`. */
int graph_out_degree(const Graph* g, int v)
{
    assert(g != NULL);
    assert((v >= 0) && (v < graph_n_nodes(g)));
    return g->out_deg[v];
}

/*Elimino tutta la memoria allocata del mio grafo*/
void graph_destroy(Graph* g)
{
    int i;

    assert(g != NULL);

    for (i = 0; i < g->n; i++) {
        Edge* edge = g->edges[i];
        while (edge != NULL) {
            Edge* next = edge->next;
            free(edge);
            edge = next;
        }
        g->edges[i] = NULL; /* e' superfluo */
    }
    free(g->edges);
    free(g->in_deg);
    free(g->out_deg);

    for (i = 0 ; i < g->righe; i++)
    {
        free(g->mappa[i]);
    }
    
    free(g->mappa);

    g->n = 0;
    g->edges = NULL;
    free(g);
}

/* Funzioni Algoritmo di Dijkstra*/

/*rilassa gli archi dei nodi analizzati*/
void relax(Edge* arcosrc, Edge** archi, int dst)

{
    Edge* temp;
   
    assert(arcosrc != NULL && archi[dst] != NULL);


    if (archi[dst]->key > ( arcosrc->key + arcosrc->weight) )
    {
        temp = archi[dst];

        while (temp != NULL)
        {
            temp->key = arcosrc->key + arcosrc->weight;
            temp->padre = arcosrc;

            temp = temp->next;
        }
    }
}

/*inizializzo tutti i nodi e archi del mio grafo*/
void inizkey(Edge** archi, int nodo)
{
    Edge* temp; 
    int i;

    for (i = 1 ; i < nodo ; i++)/*imposto i nodi del grafo con peso infinito e colore bianco*/
    {
        temp = archi[i];
        while (temp != NULL)
        {
            temp->padre = NULL;
            temp->key = INF;
            temp->Colore = BIANCO;
            temp = temp->next;
            
        }
    }

    temp = archi[0];/*solo il nodo di partenza lo imposto a key = 0 e colore giallo*/

    while (temp != NULL)
    {
        temp->key = 0 ;
        temp->padre = NULL;
        temp->Colore = GIALLO;
        temp = temp->next ;
    }
    
}


/*Funzione principale per gestire l'algoritmo di dijkstra , immettendo solo il grafo di partenza.*/
void dijkstra(Graph* g)
{
    int i , k = 0;
    Edge* temp;
    Nodoanallizzato* numero ;
    double weighttot;

    inizkey(g->edges , g->n);

    numero = buildvet(g);
      
    temp = g->edges[0];
    
    /*controllo il mio grafo fino a quando non ho concluso tutto il grafo o sono arrivato al nodo di destinazione*/
    for (i = 0 ; i < g->n ; i++)
    {

        while (temp != NULL)
        {
            relax(temp, g->edges, temp->dst);
            temp->Colore = GIALLO;

            if (g->edges[temp->dst]->Colore == BIANCO)/*se il nodo di destinazione è bianco , ovvero non analizzato, lo inserisco nella lista dei nodi da verificare*/
            {
                aggvet(numero, temp->dst, g->edges[temp->dst]->key);
                
            }
            temp = temp->next;

        }
        k = prevnum(numero , g->n );
        temp = g->edges[k];

    };
   
    free(numero);

    /*estraggo il peso totale del mio percorso mentre lo stampo*/
    weighttot = percorso(g->edges[g->n - 1] , g->n - 1 , g->ccell);
    
    puts("-1 -1");
    printf("%.f", weighttot);

}

/*Funzione che dato un punto di arrivo mi trovi la strada più breve per la partenza.
In modo ricorsivo risalgo i padri dal punto di arrivo*/
double percorso(Edge* archi, int nodo, int ccel)
{
    Edge* temp = archi;

    double weight;

    if (temp == NULL)
    {   
        return 0;
    }

    weight = percorso(temp->padre, nodo, ccel);

    printf("%d %d\n", temp->ysrc, temp->xsrc);

    if (temp->src != nodo)
    {
        weight = weight + temp->weight ;
    }
    else
    {
        weight = weight + ccel;
    }
    return weight;
}

/*Creo un vettore ordinato per gestire i nodi del mio grafo*/
Nodoanallizzato* buildvet(Graph* g)
{
    Nodoanallizzato* nuovo ;
    int i;

    nuovo = (Nodoanallizzato*)malloc(sizeof(Nodoanallizzato)* g->n);
    assert(nuovo != NULL);

    for (i = 0 ; i < g->n ; i++)
    {
        nuovo[i].key = INF;
        nuovo[i].colore = BIANCO;
        nuovo[i].num = i;
        
    }
    return nuovo;
    
}


/*Aggiorno il key del nodo analizzato*/
void aggvet(Nodoanallizzato* lista, int num, int key)
{
            lista[num].key = key;
}


/*Estraggo  il nodo con key minore , ovvero quello con un peso del persorso
ritorno il numero del nodo analizzato*/
int prevnum(Nodoanallizzato* lista, int nodi)
{
    int minimo , num;
    int i;

    num = -1;

    minimo = INF;

    for (i = 0 ; i < nodi ; i++)
    {
        if ( lista[i].colore == BIANCO && lista[i].key <= minimo)
        {
            minimo = lista[i].key;
            num = lista[i].num;
        }

    }

    assert(num != -1);

    lista[num].colore = GIALLO;/*identifico di aver analizzato il nodo*/
    
    return num;
}
