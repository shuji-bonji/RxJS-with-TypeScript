---
description: "Una spiegazione dettagliata della differenza tra Observable freddi e Observable caldi. Introduce l'indipendenza del flusso di dati per sottoscrizione, Hot con share e shareReplay, il multicasting con BehaviorSubject e i modelli di implementazione type-safe in TypeScript."
---
# Observable freddi e Observable caldi

Uno dei concetti chiave nell'uso di RxJS è la distinzione tra "Cold Observable" e "Hot Observable". La comprensione di questa distinzione è essenziale per imparare a usare in modo efficiente gli Observable.

## Perché capire Cold/Hot è importante

Se non si comprende la distinzione Cold/Hot, si incontrano i seguenti problemi.

- **Esecuzione duplicata non intenzionale** - le chiamate API vengono eseguite più volte
- **Perdite di memoria** - le sottoscrizioni non sono gestite correttamente
- **Problemi di prestazioni** - vengono ripetute elaborazioni non necessarie
- **Incoerenze dei dati** - i dati attesi non vengono ricevuti

## Differenze tra Cold e Hot (tabella di confronto)

Per prima cosa, vediamo il quadro generale.

| Voce di confronto | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Esecuzione senza sottoscrizione** | Non eseguito (eseguito solo quando si è sottoscritti) | Eseguito (emette valori anche se non sottoscritto) |
| **Tempistica emissione dati** | Inizia quando viene chiamato `subscribe()` | Inizia al momento dell'emittente (indipendentemente dalla sottoscrizione) |
| **Riutilizzo dell'esecuzione** | Nuova esecuzione ogni volta | I flussi esistenti sono condivisi da più utenti |
| **Consistenza dei dati** | Ricevere valori indipendenti per ogni sottoscrizione | Chi si iscrive a metà non riceve valori passati |
| **Casi d'uso principali** | Richieste HTTP, elaborazione asincrona | Eventi UI, WebSocket, comunicazione in tempo reale |
| **Scenari d'uso** | Quando ogni processo è indipendente | Condivisione dello stato, broadcast di eventi |

**Criteri decisionali:** Il processo deve essere rieseguito per ogni sottoscrittore? O il flusso deve essere condiviso?

## Criteri per Cold e Hot

In pratica, i seguenti criteri possono essere utilizzati per distinguere se un Observable è Cold o Hot.

| Punto di decisione | Cold | Hot |
|-------------|------|-----|
| **La logica di esecuzione viene rieseguita a ogni sottoscrizione?** | ✅ Rieseguire ogni volta | ❌ Condividere l'esecuzione |
| **I dati fluiscono prima della sottoscrizione?** | ❌ Attendere fino alla sottoscrizione | ✅ Flusso indipendentemente dalla sottoscrizione |
| **Più sottoscrizioni ricevono gli stessi dati?** | ❌ Dati indipendenti | ✅ Condividere gli stessi dati |

### Modi pratici per identificarli

Il seguente test può essere facilmente determinato.

```typescript
const observable$ = /* Observable da testare */;

observable$.subscribe(/* sottoscrizione 1 */);
observable$.subscribe(/* sottoscrizione 2 */);

// ✅ Cold: console.log all'interno dell'Observable viene eseguito 2 volte
//         (la logica di esecuzione viene rieseguita per ogni sottoscrizione)
// ✅ Hot:  console.log all'interno dell'Observable viene eseguito solo 1 volta
//         (l'esecuzione è condivisa)
```

**Esempi specifici:**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold: inizio esecuzione');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Sottoscrizione 1:', v));
cold$.subscribe(v => console.log('Sottoscrizione 2:', v));
// Output:
// Cold: inizio esecuzione  ← 1a volta
// Sottoscrizione 1: 0.123...
// Cold: inizio esecuzione  ← 2a volta (rieseguito)
// Sottoscrizione 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Sottoscrizione 1:', v));
hot$.subscribe(v => console.log('Sottoscrizione 2:', v));
hot$.next(1); // L'emissione dati avviene solo 1 volta
// Output:
// Sottoscrizione 1: 1
// Sottoscrizione 2: 1  ← Stessi dati condivisi
```

## Tabella di classificazione Cold/Hot per Creation Function

Classifica Cold/Hot per tutte le principali Creation Function. Ciò consente di vedere a colpo d'occhio quale funzione produce quale Observable.

| Categoria | Creation Function | Cold/Hot | Note |
|---------|-------------------|----------|------|
| **Creazione base** | `of()` | ❄️ Cold | Riemissione valore per ogni sottoscrizione |
| | `from()` | ❄️ Cold | Rieseguire array/Promise per ogni sottoscrizione |
| | `fromEvent()` | ❄️ Cold | Aggiungere un listener separato per ogni sottoscrizione [^fromEvent] |
| | `interval()` | ❄️ Cold | Timer indipendente per ogni sottoscrizione |
| | `timer()` | ❄️ Cold | Timer indipendente per sottoscrizione |
| **Generazione loop** | `range()` | ❄️ Cold | Range rigenerato per sottoscrizione |
| | `generate()` | ❄️ Cold | Loop rieseguito per ogni sottoscrizione |
| **Comunicazione HTTP** | `ajax()` | ❄️ Cold | Nuova richiesta HTTP per ogni sottoscrizione |
| | `fromFetch()` | ❄️ Cold | Nuova richiesta Fetch per sottoscrizione |
| **Combinazione** | `concat()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| | `merge()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| | `combineLatest()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| | `zip()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| | `forkJoin()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| **Selezione/Partizione** | `race()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| | `partition()` | ❄️ Cold | Eredita le proprietà dell'Observable originale [^combination] |
| **Ramificazione condizionale** | `iif()` | ❄️ Cold | Assume le proprietà dell'Observable selezionato dalla condizione |
| | `defer()` | ❄️ Cold | Esegue una funzione factory per ogni sottoscrizione |
| **Controllo** | `scheduled()` | ❄️ Cold | Eredita le proprietà dell'Observable originale |
| | `using()` | ❄️ Cold | Crea una risorsa per ogni sottoscrizione |
| **Subject** | `new Subject()` | 🔥 Hot | Sempre Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Sempre Hot |
| | `new ReplaySubject()` | 🔥 Hot | Sempre Hot |
| | `new AsyncSubject()` | 🔥 Hot | Sempre Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Condivide connessione WebSocket |

[^fromEvent]: `fromEvent()` è Cold perché aggiunge un event listener indipendente per ogni sottoscrizione; tuttavia, è facilmente fraintendibile come Hot, perché l'evento stesso si verifica indipendentemente dalla sottoscrizione.

[^combination]: Le Creation Function di combinazione sono Cold se l'Observable di partenza è Cold e Hot se è Hot. Di solito, gli Observable Cold vengono combinati tra loro.

> [!IMPORTANT] Principi chiave
> **Quasi tutte le Creation Function producono Cold.**
> Generano Hot solo:
> - Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()
>
> Questi sono gli unici.

## Cold Observable

### Caratteristiche

- **Un nuovo flusso di dati viene creato ogni volta che viene sottoscritto**
- **La pubblicazione dei dati non inizia finché non viene sottoscritto (esecuzione lazy)**
- **Tutti i sottoscrittori ricevono tutti i dati dall'inizio dell'Observable**

Cold Observable crea un nuovo contesto di esecuzione a ogni sottoscrizione.
Questo è adatto per le richieste HTTP, l'elaborazione asincrona e così via, dove è necessario un nuovo processo ogni volta.

### Esempio di codice

```typescript
import { Observable } from 'rxjs';

// Esempio di Cold Observable
const cold$ = new Observable<number>(subscriber => {
  console.log('Creazione data source - nuova sottoscrizione');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// Prima sottoscrizione
console.log('--- Prima sottoscrizione ---');
cold$.subscribe(value => console.log('Sottoscrittore 1:', value));

// Seconda sottoscrizione (vengono generati dati diversi)
console.log('--- Seconda sottoscrizione ---');
cold$.subscribe(value => console.log('Sottoscrittore 2:', value));
```

#### Risultato dell'esecuzione
```sh
--- Prima sottoscrizione ---
Creazione data source - nuova sottoscrizione
Sottoscrittore 1: 0.259632...
--- Seconda sottoscrizione ---
Creazione data source - nuova sottoscrizione  ← Rieseguito
Sottoscrittore 2: 0.744322...  ← Valore diverso
```

> [!TIP] Punti importanti
> Ogni sottoscrizione eseguirà "Creazione data source" e genererà valori diversi.

### Cold Observable comuni (come identificarli)

I seguenti Observable sono normalmente Cold.

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Operatori temporali
interval(1000)                 // Cold
timer(1000)                    // Cold

// Richieste HTTP
ajax('/api/users')             // Cold
```

> [!TIP] Regole
> Le Creation Function, gli operatori temporali e le richieste HTTP sono fondamentalmente Cold

## Hot Observable

### Caratteristiche

- **Emette valori anche se non è sottoscritto (funziona con o senza sottoscrizione)**
- **Riceve dati solo dal punto di inizio della sottoscrizione in poi**
- **Una fonte di dati condivisa da più sottoscrittori**

Hot Observable, in cui la tempistica della pubblicazione del flusso è indipendente dalla sottoscrizione e i sottoscrittori si uniscono a metà del flusso.

### Esempio di codice

```typescript
import { Subject } from 'rxjs';

// Esempio di Hot Observable (usando Subject)
const hot$ = new Subject<number>();

// Prima sottoscrizione
console.log('--- Sottoscrittore 1 inizio ---');
hot$.subscribe(value => console.log('Sottoscrittore 1:', value));

// Emissione dati
hot$.next(1);
hot$.next(2);

// Seconda sottoscrizione (sottoscrizione tardiva)
console.log('--- Sottoscrittore 2 inizio ---');
hot$.subscribe(value => console.log('Sottoscrittore 2:', value));

// Ulteriore emissione dati
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Risultato dell'esecuzione
```sh
--- Sottoscrittore 1 inizio ---
Sottoscrittore 1: 1
Sottoscrittore 1: 2
--- Sottoscrittore 2 inizio ---
Sottoscrittore 1: 3
Sottoscrittore 2: 3  ← Sottoscrizione 2 si unisce dal 3 (1, 2 non ricevuti)
Sottoscrittore 1: 4
Sottoscrittore 2: 4
```

> [!TIP] Punti importanti
> Il sottoscrittore 2 si è unito a metà del processo e non riceverà i valori precedenti (1, 2).

### Hot Observable comuni (come identificarli)

I seguenti Observable sono sempre Hot.

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Subject (sempre Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (sempre Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Regole
> **Solo i Subject e webSocket() generano Hot**

> [!WARNING] fromEvent() è Cold
> `fromEvent(button, 'click')` è facilmente frainteso come Hot, ma in realtà è **Cold**. Aggiunge un event listener separato per ogni sottoscrizione. L'evento stesso si verifica indipendentemente dalla sottoscrizione, ma ogni sottoscrittore ha un listener indipendente.

## Come convertire un Cold Observable in Hot

In RxJS, il mezzo principale per convertire un Cold Observable in Hot è il seguente:

- `share()` - semplice conversione a Hot (consigliato)
- `shareReplay()` - cache dei valori passati e conversione a Hot
- ~~`multicast()`~~ - deprecato (deprecato in RxJS v7, rimosso in v8)

### Operatore share()

`share()` è il modo più comune per convertire un Cold Observable in un Hot Observable.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// Simula una chiamata HTTP
const makeHttpRequest = () => {
  console.log('Chiamata HTTP eseguita!');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (senza condivisione)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Sottoscrittore 1:', val));
cold$.subscribe(val => console.log('Sottoscrittore 2:', val));
// → La chiamata HTTP viene eseguita 2 volte

// ✅ Hot Observable (usando share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Sottoscrittore condiviso 1:', val));
shared$.subscribe(val => console.log('Sottoscrittore condiviso 2:', val));
// → La chiamata HTTP viene eseguita solo 1 volta, il risultato è condiviso
```

**Risultato dell'esecuzione (Cold):**
```sh
Chiamata HTTP eseguita!  ← 1a volta
Sottoscrittore 1: 0
Chiamata HTTP eseguita!  ← 2a volta (duplicato!)
Sottoscrittore 2: 0
...
```

**Risultato dell'esecuzione (Hot):**
```sh
Chiamata HTTP eseguita!  ← Solo 1 volta
Sottoscrittore condiviso 1: 0
Sottoscrittore condiviso 2: 0  ← Condividono lo stesso stream
...
```

> [!NOTE] Casi d'uso
> - Utilizzare lo stesso risultato API in più componenti
> - Evitare effetti collaterali duplicati (es. chiamate HTTP)

### Operatore shareReplay()

`shareReplay()` è un'estensione di `share()` che **memorizza nella cache** i valori passati e li ripropone ai nuovi sottoscrittori.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Cache degli ultimi 2 valori
);

// Prima sottoscrizione
request$.subscribe(val => console.log('Sottoscrittore 1:', val));

// Seconda sottoscrizione dopo 3.5 secondi (dopo il completamento dello stream)
setTimeout(() => {
  console.log('--- Sottoscrittore 2 inizio (dopo completamento) ---');
  request$.subscribe(val => console.log('Sottoscrittore 2:', val));
}, 3500);
```

#### Risultati dell'esecuzione
```sh
Sottoscrittore 1: 0
Sottoscrittore 1: 1
Sottoscrittore 1: 2
--- Sottoscrittore 2 inizio (dopo completamento) ---
Sottoscrittore 2: 1  ← Valori in cache (ultimi 2)
Sottoscrittore 2: 2  ← Valori in cache
```

> [!NOTE] Casi d'uso
> - Cache dei risultati API
> - Condividere lo stato iniziale (solo l'ultimo in cache)
> - Fornire dati storici agli abbonati ritardatari

> [!WARNING] Note su shareReplay
> `shareReplay()` continua a mantenere la cache anche quando le sottoscrizioni vanno a 0, il che può causare perdite di memoria. Per ulteriori informazioni, vedere [Capitolo 10: Uso improprio di shareReplay](/it/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用).

### Informazioni su multicast()

> [!NOTE]
> `multicast()` è flessibile, ma è stato deprecato in RxJS v7 e rimosso in v8. Utilizzare ora `share()` o `shareReplay()`. Per ulteriori informazioni, vedere [descrizione dell'operatore share()](/it/guide/operators/multicasting/share).

## Esempio pratico: servizio di cache API

Uno schema comune nelle applicazioni reali: quando diversi componenti hanno bisogno degli stessi dati API.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Semplice servizio di cache
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Se esiste la cache, restituiscila
    if (this.cache$) {
      console.log('Restituito dalla cache');
      return this.cache$;
    }

    // Crea una nuova richiesta e mettila in cache
    console.log('Esecuzione nuova richiesta');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Cancella la cache in caso di errore
        return throwError(() => err);
      }),
      shareReplay(1)  // Cache dell'ultimo risultato
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Simula una richiesta API reale
    return of([
      { id: 1, name: 'Mario Rossi' },
      { id: 2, name: 'Anna Bianchi' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Dati ricevuti dall\'API'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache cancellata');
  }
}

interface User {
  id: number;
  name: string;
}

// Esempio di utilizzo
const userService = new UserService();

// Componente 1: richiede dati
userService.getUsers().subscribe(users =>
  console.log('Componente 1:', users)
);

// Componente 2: richiede dati dopo 2 secondi
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Componente 2:', users)
  );
}, 2000);

// Cancella la cache e richiedi di nuovo
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Componente 3:', users)
  );
}, 4000);
```

#### Risultati dell'esecuzione
```sh
Esecuzione nuova richiesta
Dati ricevuti dall'API
Componente 1: [{id: 1, name: 'Mario Rossi'}, {id: 2, name: 'Anna Bianchi'}]
Restituito dalla cache  ← Nessuna chiamata API
Componente 2: [{id: 1, name: 'Mario Rossi'}, {id: 2, name: 'Anna Bianchi'}]
Cache cancellata
Esecuzione nuova richiesta  ← Nuova chiamata API
Dati ricevuti dall'API
Componente 3: [{id: 1, name: 'Mario Rossi'}, {id: 2, name: 'Anna Bianchi'}]
```

**Punti:**
- Cache dell'ultima risposta con `shareReplay(1)`
- Più componenti condividono i dati (solo una chiamata API)
- La cache viene distrutta correttamente in caso di errore o quando viene cancellata

## Quando usare

<div class="comparison-cards">

::: tip Cold
#### Usare quando
- Quando ogni sottoscrittore ha bisogno di un proprio insieme di dati
- Quando si rappresenta un processo o un'azione appena avviati
- Quando la duplicazione degli effetti collaterali non è un problema

#### Esempio
- Inviare una nuova richiesta POST per ogni invio di un modulo
- Sono necessari timer diversi per ogni utente
- Eseguire calcoli indipendenti su ogni sottoscrizione
:::

::: tip Hot
#### Usare quando
- Quando si condividono dati tra più componenti
- Quando si desidera risparmiare risorse (es. ridurre il numero di chiamate HTTP)
- Quando si rappresentano flussi di eventi
- Gestione dello stato o comunicazione tra servizi

#### Esempio
- Informazioni di configurazione condivise in tutta l'applicazione
- Stato di accesso dell'utente
- Messaggi in tempo reale (WebSocket)
- Eventi DOM (clic, scorrimento, ecc.)
:::

</div>

## Riepilogo

La comprensione e il corretto utilizzo di Cold Observable e Hot Observable è un'abilità importante per costruire applicazioni RxJS efficienti.

::: tip Punti chiave
- **Cold Observable**: uno stream che inizia a funzionare solo quando viene sottoscritto (esecuzione indipendente per sottoscrizione)
- **Hot Observable**: condivide uno stream già in esecuzione (più sottoscrizioni, stessa esecuzione)
- **share()**: il modo più semplice per convertire Cold in Hot
- **shareReplay()**: memorizza nella cache i valori passati e li converte in Hot (utile per condividere i risultati delle API)
:::

::: tip Criteri per le decisioni di progettazione
- È necessario condividere i dati tra più sottoscrittori?
- I valori passati devono essere memorizzati nella cache e forniti ai nuovi sottoscrittori?
- Come verranno gestiti gli effetti collaterali duplicati (es. richieste HTTP)?
:::

Sulla base di queste considerazioni, la scelta del tipo di Observable e dell'operatore giusto può aiutare a costruire un'applicazione reattiva efficiente e robusta.

## Sezioni correlate

- **[Operatore share()](/it/guide/operators/multicasting/share)** - Spiegazione dettagliata di share()
- **[Uso improprio di shareReplay](/it/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - Errori comuni e rimedi
- **[Subject](/it/guide/subjects/what-is-subject)** - Capire i Subject Hot

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
