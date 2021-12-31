MARCHING CUBES & EXTENDED MARCHING CUBES
===================================================================================
In breve le classi coinvolte sono 3 e sono:
	* MerchingCubes ed ExtendedMarchingCubes
			processano una cella alla volta, aggiungendo per ogni chiamata a ProcessCell 
			l'insieme di triangoli approssimante la superficie che interseca la cella
	* Walker
			gestisce l'attraversamento del volume, servendo le chiamate effettuate dagli
			algoritmi di estrazione della superficie al volume e cachandone i risultato
	*	Volume
			conosce come calcolare il campo scalare all'interno del volume da processare 
			e come calcolare le intersezioni superficie/segmenti.
			

DESCRIZIONE
====================================================================================
	Le classi che implementano gli algoritmi MarchingCubes ed ExtendedMarchingCubes
sono state implementate così da risultare quanto più generiche possibile: ogni chiamata
al metodo ProcessCell(Point3i p1, Point3i p2) analizza la cella del volume individuata 
dai due punti p1 e p2 e l'analisi di questa cella si conclude esattamente al ritorno da questa 
chiamata: nel caso infatti la superficie da estrarre attraversi questa cella, all'interno 
di questa stessa chiamata la mesh viene aggiornata con un opportuno insieme di triangoli.
L'assunzione alla base di questa astrazione è l'esistenza di altri due entità, il Walker
ed il Volume; sebbene sulla loro implementazione è lasciata la più completa libertà, è utile
precisare in quale relazione essi stiano rispetto agli algoritmi di estrazione di superfici.

Un esempio che riassume quanto qui esposto è incluso nella libreria: vd. vcg/apps/test/extractors.
	
VOLUME
====================================================================================
	Gli algoritmi di estrazione di superfici risalgono alla superficie utilizzando i valori
di un campo scalare definito sul volume da processare campionato sui vertici di una qualche 
griglia. Questo campo scalare sarà generalmente diverso a seconda del tipo di applicazione.
Il Volume è appunto quella classe che racchiude il campo scalare e di cui ne conosce le proprietà.
In realtà, all'interno dell'algoritmo di estrazione di superfici, non esiste alcun collegamento
esplicito con il Volume: tutte le sue chiamate sono rivolte al Walker. Questo perché
(per motivi che saranno esposti successivamente a proposito del Walker) il Walker potrebbe già 
possedere il valore del campo calcolato in un dato punto, che evita così di farselo ricalcolare 
nuovamente dal Volume: Similmente, quando viene chiesto di calcolare il punto di intersezione 
della superficie con un segmento, se il Walker dispone già di questa informazione, la restituisce
all'algoritmo di estrazione di superfici, altrimenti il calcolo verrà effettuato dal Volume: il 
punto ottenuto verrà restituito al Walker che potrà quindi soddisfare la richiesta iniziale. 
Il motivo per cui si è scelto di frapporre un Walker fra gli algoritmi di estrazione di superfici
ed il Volume è esclusivamente di ottimizzazione. L'idea di fondo è che il Walker è quell'oggetto
attrverso cui avviene la visita del volume: per ogni cella del volume, esso effettua la chiamata
ProcessCell. Conoscendo l'ordine di visita, il Walker è anche la classe candidata ad implementare
le politiche di caching, in quanto sa esattamente da che momento può essere utile una certa 
informazione e per quanto a lungo può essere conveniente mantenerla prima di liberarsene.

WALKER
====================================================================================
Poiché la politica di visita del volume è realizzata all'interno del walker, 
è opportuno che sempre all'interno del walker vengano realizzate le politiche 
di caching rivolte ad ottimizzare l'esecuzione degli algoritmi MC ed EMC. Durante
il processing di ogni cella questi algoritmi possono chiamare le seguenti funzioni
del Walker:
																						MC		 EMC
							------------------------------------------
							V(i, j, k)										X				X
							GetXIntercept(p1, p2, v)			X				X
							GetYIntercept(p1, p2, v)			X				X
							GetZIntercept(p1, p2, v)			X				X
							Exist(p1, p2, v)							X

const float V(int i, int j, int k) const 
	La superficie che attraversa ogni cella viene ricavata dall'algoritmo di estrazione
	analizzando il valore del campo sugli otto spigoli di ogni voxel del volume; 
	per ogni voxel, il valore del campo sui suoi otto spigoli vengono richiesti 
	dall'algoritmo di estrazione al walker: se questo valore è già stato calcolato 
	e cachato, il walker restituisce direttamente tale valore; altrimenti il valore 
	del campo in questo spigolo viene calcolato (eventualmente cachato) e restituito
	al walker. In questo modo il valoro del campo ad ogni punto viene calcolato una
	sola volta anziché 8, questo puo' essere molto utile nel caso si utilizzi dataset 
	volumetrici mantenuti implicitamente.

void GetXIntercept(Point3i p1, Point3i p2, VertexPointer v)
void GetYIntercept(Point3i p1, Point3i p2, VertexPointer v)
void GetZIntercept(Point3i p1, Point3i p2, VertexPointer v)
	Dall'analisi del valore del campo agli spigoli di un dato voxel, l'algoritmo di 
	estrazione ha rilevato che la superficie interseca lo spigolo avente estremi p1 
	e p2(a seconda dell'orientazione di questo spigolo, viene chiamato uno dei tre 
	metodi): al termine di una di queste chiamate, v deve puntare al vertice della
	mesh (di coordinate comprese fra p1 e p2) attraverso cui passa la superficie. 
	Se questo vertice è stato già calcolato ed inserito nella mesh, il walker deve 
	risalire a tale vertice e memorizzarne in v il suo puntatore. Altrimenti deve provvedere
	ad aggiugnere un nuovo vertice ed a calcolare la sua posizione; v deve puntare 
	in questo caso al vertice appena inserito.
	Il motivo per cui questo calcolo non viene implementato direttamente negli algoritmi
	di estrazione (possibile per es. attraverso interpolazione lineare del valore 
	del campo nei punti p1 e p2) è che questo calcolo può essere fatto in maniere
	molto più precisa conoscendo come il campo viene calcolato, essendo infatti ciò 
	dipendente dall'applicazione.
	
bool Exist(Point3i p1, Point3i p2, VertexPointer v)
	Questo metodo viene chiamato solamente all'interno dell'algoritmo MarchingCubes.
	A differenza dei tre motodi precedenti, in questo caso si vuole sapere solamente
	se esiste già un vertice tra i punti p1 e p2: nel caso tale vertice esista, Exist
	deve resituire true ed v deve puntare a tale vertice; se invece tale vertice non 
	esiste, Exist deve restituire false e v deve prendere il valore NULL.
	NB: nel caso in cui il vertice non esiste, alla mesh non deve essere 
		aggiunto alcun nuovo vertice.

