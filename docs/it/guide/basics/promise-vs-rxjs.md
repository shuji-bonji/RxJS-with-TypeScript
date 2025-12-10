---
description: "Comprendere le differenze tra Promise e RxJS e imparare a usarli appropriatamente. Promise è specializzato per operazioni asincrone singole e viene eseguito immediatamente, mentre RxJS fornisce elaborazione di stream con valutazione lazy che può gestire valori multipli. Le differenze tra cancellazione, retry e trasformazione sono spiegate e confrontate in TypeScript."
---

# Differenze tra Promise e RxJS

## Panoramica

I due strumenti principali per l'elaborazione asincrona in JavaScript/TypeScript sono **Promise** e **RxJS (Observable)**. A volte vengono utilizzati per scopi simili, ma hanno filosofie di progettazione e casi d'uso molto diversi.

Questa pagina fornisce informazioni per comprendere le differenze tra Promise e RxJS e decidere quale utilizzare.

## Differenze fondamentali

| Elemento | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Standardizzazione** | Standard JavaScript (ES6/ES2015) | Libreria di terze parti |
| **Valori emessi** | Valore singolo | Zero o più valori multipli |
| **Valutazione** | Eager (esecuzione immediata alla creazione) | Lazy (esecuzione alla sottoscrizione) |
| **Cancellazione** | Non possibile[^1] | Possibile (`unsubscribe()`) |
| **Riutilizzo** | Non possibile (risultato una sola volta) | Possibile (sottoscrivibile più volte) |
| **Costo di apprendimento** | Basso | Alto (richiede comprensione degli operatori) |
| **Casi d'uso** | Elaborazione asincrona singola | Elaborazione di stream complessi |

[^1]: È possibile cancellare elaborazioni basate su Promise (come fetch) usando AbortController, ma la specifica di Promise stessa non include funzionalità di cancellazione.

## Confronto codice: Singola elaborazione asincrona

### Promise

```ts
// Promise viene eseguita immediatamente alla creazione (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise inizia l'esecuzione **nell'istante in cui viene definita** (valutazione Eager).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable non viene eseguito finché non viene sottoscritto (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() restituisce una Promise, quindi si usa switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// Viene eseguito solo quando sottoscritto
observable$.subscribe(data => console.log(data));
```

RxJS **non viene eseguito finché non viene chiamato `subscribe()`** (valutazione Lazy). Sottoscrivere lo stesso Observable più volte comporta esecuzioni indipendenti, e l'elaborazione può essere interrotta con `unsubscribe()`.

> [!TIP]
> **Utilizzo nella pratica**
> - Elaborazioni singole da eseguire immediatamente → Promise
> - Elaborazioni da eseguire al momento necessario o più volte → RxJS

## Confronto codice: Gestione di valori multipli

Una delle maggiori differenze tra Promise e RxJS è il numero di valori che possono essere emessi. Promise può restituire un solo valore, mentre RxJS può emettere più valori in ordine cronologico.

### Non possibile con Promise

Promise può essere risolto **una sola volta**.

```ts
// Promise può restituire solo un valore singolo
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Questo valore viene ignorato
  resolve(3); // Anche questo valore viene ignorato
});

promise.then(value => console.log(value));
// Output: 1 (solo il primo valore)
```

Una volta che il valore viene determinato dalla prima `resolve()`, le successive `resolve()` vengono ignorate.

### Possibile con RxJS

Observable può emettere valori **quante volte si vuole**.

```ts
import { Observable } from 'rxjs';

// Observable può emettere valori multipli
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Output: 1, 2, 3
```

Ogni volta che viene chiamato `next()`, un valore viene consegnato al sottoscrittore. Dopo che tutti i valori sono stati emessi, `complete()` segnala il completamento. Questa caratteristica permette di gestire naturalmente dati che cambiano nel tempo, come comunicazioni in tempo reale, streaming di dati ed elaborazione continua di eventi.

> [!NOTE]
> **Esempi di applicazione pratica**
> - Ricezione messaggi WebSocket
> - Elaborazione sequenziale input da tastiera
> - Stream di eventi dal server (SSE)
> - Monitoraggio continuo dati dei sensori

## Confronto cancellazione

La possibilità di cancellare elaborazioni a lunga durata o elaborazioni asincrone non più necessarie è importante per la gestione delle risorse e l'esperienza utente. Promise e RxJS hanno grandi differenze nelle capacità di cancellazione.

### Promise (Non cancellabile)

Promise **non ha una funzione di cancellazione standard**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Completato'), 3000);
});

promise.then(result => console.log(result));
// Non c'è un modo standard per cancellare questa elaborazione
```

Una volta iniziata l'esecuzione, non può essere fermata fino al completamento, causando memory leak e degradazione delle prestazioni.

> [!WARNING]
> **Riguardo AbortController**
> Le Web API come `fetch()` possono essere cancellate usando `AbortController`, ma questa non è una caratteristica di Promise stesso, bensì un meccanismo fornito da singole API. Non può essere usato per tutte le elaborazioni asincrone.

### RxJS (Cancellabile)

RxJS può essere cancellato in qualsiasi momento con **`unsubscribe()`**.

```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Completato')
);

// Cancella dopo 1 secondo
setTimeout(() => {
  subscription.unsubscribe(); // Cancellazione
  console.log('Cancellato');
}, 1000);
// Output: Cancellato ("Completato" non viene stampato)
```

Cancellare la sottoscrizione ferma immediatamente l'elaborazione in corso e previene memory leak.

> [!TIP]
> **Esempi di utilizzo della cancellazione nella pratica**
> - Cancellare richieste HTTP quando l'utente lascia lo schermo
> - Scartare risultati di vecchie query di ricerca ed elaborare solo quelle più recenti (`switchMap`)
> - Cancellare automaticamente tutti gli Observable quando un componente viene distrutto (pattern `takeUntil`)

## Quale scegliere?

La scelta tra Promise e RxJS dipende dalla natura dell'elaborazione e dai requisiti del progetto. Usate i seguenti criteri per scegliere lo strumento appropriato.

### Quando scegliere Promise

Promise è appropriato quando si applicano le seguenti condizioni:

| Condizione | Motivo |
|------|------|
| Singola elaborazione asincrona | Una richiesta API, una lettura file, ecc. |
| Workflow semplice | `Promise.all`, `Promise.race` sono sufficienti |
| Piccoli progetti | Si vuole minimizzare le dipendenze |
| Solo API standard | Si vogliono evitare librerie esterne |
| Codice per principianti | Si vuole ridurre il costo di apprendimento |

#### Singola richiesta API:

```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Impossibile recuperare i dati utente');
  }
  return response.json();
}

// Esempio di utilizzo
getUserData('1').then(user => {
  console.log('Nome utente:', user.name);
  console.log('Email:', user.email);
});
```

Questo codice è un pattern tipico per recuperare informazioni di un singolo utente. Usando `async/await`, può essere scritto in modo leggibile come codice sincrono. Anche la gestione degli errori può essere unificata con `try/catch`, in modo semplice e intuitivo.

#### Esecuzione parallela di più elaborazioni asincrone:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Esempio di utilizzo
loadAllData().then(([users, posts]) => {
  console.log('Numero utenti:', users.length);
  console.log('Numero post:', posts.length);
});
```

Usando `Promise.all()`, più richieste API possono essere eseguite in parallelo, aspettando che tutte vengano completate. Questo è molto utile per il caricamento iniziale dei dati. È importante notare che se anche una sola fallisce, l'intera operazione risulta in errore, ma la sua semplicità la rende facile da capire e mantenere.

### Quando scegliere RxJS

RxJS è appropriato quando si applicano le seguenti condizioni:

| Condizione | Motivo |
|------|------|
| Elaborazione continua di eventi | Movimento mouse, input tastiera, WebSocket, ecc. |
| Elaborazione complessa di stream | Combinazione e trasformazione di più sorgenti di eventi |
| Cancellazione necessaria | Controllo fine della gestione risorse |
| Retry/Timeout | Gestione flessibile degli errori |
| Progetti Angular | RxJS è integrato nel framework |
| Dati in tempo reale | I dati vengono aggiornati continuamente |

#### Esempio concreto

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Ricerca in tempo reale (autocompletamento)
if (!searchInput) throw new Error('Campo di ricerca non trovato');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Attende 300ms prima di elaborare
  distinctUntilChanged(),         // Elabora solo quando il valore cambia
  switchMap(query =>              // Esegue solo la richiesta più recente
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Risultati ricerca:', results.items); // GitHub API memorizza i risultati nella proprietà items
});
```

Questo esempio è un caso tipico in cui si dimostra il vero valore di RxJS. Monitora l'input dell'utente, riduce le richieste inutili con un tempo di attesa di 300ms, elabora solo quando il valore cambia e scarta automaticamente i risultati delle vecchie richieste attivando solo quelle più recenti (`switchMap`).

> [!IMPORTANT]
> **Perché è difficile con solo Promise**
> - Il debounce (controllo dell'input continuo) deve essere implementato manualmente
> - La cancellazione delle vecchie richieste deve essere gestita autonomamente
> - Memory leak se si dimentica di pulire gli event listener
> - Necessità di tracciare più stati (timer, flag, gestione richieste) simultaneamente
>
> Con RxJS, tutto questo può essere realizzato in modo dichiarativo e in poche righe.

## Interoperabilità tra Promise e RxJS

Promise e RxJS non sono esclusivi e possono essere convertiti e combinati. È utile quando si vuole integrare codice esistente basato su Promise in una pipeline RxJS, o viceversa quando si vogliono usare Observable in codice esistente basato su Promise.

## Convertire Promise in Observable

RxJS fornisce diversi modi per convertire Promise esistenti in Observable.

### Conversione con `from`

Il metodo più comune è usare `from`.

```ts
import { from } from 'rxjs';

// Crea una Promise
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Converte in Observable con from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Completato')
});
```

`from()` emette un valore quando la Promise viene risolta e immediatamente `complete`. Se si verifica un errore, viene inviata una notifica `error`. Questa conversione permette di applicare liberamente gli operatori RxJS (`map`, `filter`, `retry`, ecc.) anche ai dati derivati da Promise.

### Conversione con `defer` (valutazione lazy)

`defer` ritarda la creazione della Promise fino alla sottoscrizione.

```ts
import { defer } from 'rxjs';

// La Promise non viene creata finché non viene sottoscritto
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Crea una nuova Promise ad ogni sottoscrizione
observable$.subscribe(data => console.log('1a volta:', data));
observable$.subscribe(data => console.log('2a volta:', data));
```

Questo metodo è utile quando si vuole creare una nuova Promise ad ogni sottoscrizione.

## Convertire Observable in Promise

Si può estrarre un solo valore da un Observable e trasformarlo in Promise.

### `firstValueFrom` e `lastValueFrom`

Da RxJS 7, sono raccomandate le seguenti due funzioni:

| Funzione | Comportamento |
|------|------|
| `firstValueFrom` | Restituisce il primo valore come Promise |
| `lastValueFrom` | Restituisce l'ultimo valore al completamento come Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Ottiene il primo valore come Promise
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Ottiene l'ultimo valore come Promise
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Se l'Observable completa prima di emettere un valore, per default risulta in errore. Questo può essere evitato specificando un valore predefinito.

> [!WARNING]
> `toPromise()` è deprecato. Usare invece `firstValueFrom()` o `lastValueFrom()`.

> [!TIP]
> **Linee guida per la scelta**
> - **`firstValueFrom()`**: Quando serve solo il primo valore (es. risultato autenticazione login)
> - **`lastValueFrom()`**: Quando serve il risultato finale dopo aver elaborato tutti i dati (es. risultati aggregati)

## Esempio pratico: Combinare entrambi

Nelle applicazioni reali, è comune usare Promise e RxJS in combinazione.

> [!WARNING] Considerazioni pratiche
> Mescolare Promise e Observable **può facilmente diventare un anti-pattern se i confini del design non sono chiari**.
>
> **Problemi comuni:**
> - Diventa non cancellabile
> - Separazione della gestione errori
> - `await` dentro `subscribe` (particolarmente pericoloso)
> - Recupero parallelo degli stessi dati con Promise e Observable
>
> Per dettagli, vedere **[Capitolo 10: Anti-pattern del mixing Promise e Observable](/it/guide/anti-patterns/promise-observable-mixing)**.

### Invio form e chiamate API

Esempio di cattura dell'evento di invio form dell'utente con RxJS e invio al server usando Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Invio form basato su Promise
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Invio fallito');
  }
  return response.json();
}

// Gestione stream eventi con RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Invia';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Pulsante invio non trovato');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Converte funzione Promise in Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Errore invio:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Invio riuscito');
  } else {
    console.log('Invio fallito');
  }
});
```

Ogni volta che il pulsante di invio viene cliccato, viene avviato un nuovo processo di invio, ma **i nuovi invii vengono ignorati durante l'invio in corso**.

In questo esempio, l'uso di `exhaustMap` previene richieste duplicate durante l'invio.

### Autocompletamento ricerca

Esempio di monitoraggio delle modifiche ai valori del form di input ed esecuzione di ricerche API.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Funzione API basata su Promise
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Ricerca fallita');
  }
  return response.json();
}

// Gestione stream eventi con RxJS
const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Campo di ricerca non trovato');

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  switchMap(event => {
    const query = (event.target as HTMLInputElement).value;
    // Converte funzione Promise in Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Restituisce risultato vuoto in caso di errore
  })
).subscribe(result => {
  console.log('Risultati ricerca:', result.items);
  console.log('Totale:', result.total_count);
});
```

> [!TIP]
> **Design con separazione delle responsabilità**
>
> - **RxJS**: Responsabile del controllo eventi (debounce, switchMap, ecc.)
> - **Promise**: Responsabile delle richieste HTTP (async/await)
> - **`from()`**: Fa da ponte tra i due
>
> Usando ogni tecnologia nel posto giusto, la leggibilità e manutenibilità del codice migliorano.

## Vantaggi e svantaggi

### Promise

<div class="comparison-cards">

::: tip Vantaggi
- Nessuna dipendenza richiesta essendo standard JavaScript
- Codice intuitivo e leggibile con `async/await`
- Basso costo di apprendimento
- Elaborazione semplice di singoli task
:::

::: danger Svantaggi
- Non può gestire valori multipli
- Nessuna funzione di cancellazione
- Non adatto per elaborazione continua di stream
- Difficile elaborare eventi complessi
:::

</div>

### RxJS

<div class="comparison-cards">

::: tip Vantaggi
- Può gestire valori multipli in serie temporali
- Elaborazione complessa possibile con ricchi operatori
- Cancellazione (`unsubscribe`) semplice
- Implementazione flessibile di gestione errori e retry
- Dichiarativo e facile da testare
:::

::: danger Svantaggi
- Alto costo di apprendimento
- Richiede dipendenza da libreria
- Overhead presente (eccessivo per piccoli progetti)
- Debug può essere difficile
:::

</div>

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

## Aree dove RxJS eccelle particolarmente

RxJS è particolarmente potente nelle seguenti aree. Può risolvere elegantemente requisiti complessi che sarebbero difficili da realizzare con solo Promise.

| Area | Esempi concreti | Confronto con Promise |
|------|--------|----------------|
| **Comunicazione in tempo reale** | WebSocket, SSE, chat, aggiornamenti prezzi azioni | Promise è solo per comunicazioni singole. Non adatto per elaborazione continua di messaggi |
| **Controllo input utente** | Autocompletamento ricerca, validazione form | debounce, distinctUntilChanged, ecc. sono integrati |
| **Combinazione di più sorgenti** | Combinazione di criteri di ricerca × ordine × filtro | Descrivibile concisamente con combineLatest, withLatestFrom |
| **Supporto offline** | PWA, monitoraggio stato rete, risincronizzazione automatica | Controllo flessibile dei retry con retry, retryWhen |
| **API Streaming** | OpenAI, output sequenziale token risposta AI | Elaborazione dati continui in tempo reale |
| **Controllo cancellazione** | Interruzione elaborazioni lunghe, scarto vecchie richieste | Cancellazione immediata con unsubscribe() |

> [!NOTE]
> Per dettagli sulle aree di utilizzo di RxJS, vedere anche [Cos'è RxJS - Casi d'uso](./what-is-rxjs.md#casi-duso).

## Riepilogo

| Scopo | Raccomandazione | Motivo |
|------|------|------|
| Singola richiesta HTTP | Promise (`async/await`) | Semplice, leggibile, API standard |
| Elaborazione eventi input utente | RxJS | Necessario controllo debounce, distinct, ecc. |
| Dati in tempo reale (WebSocket) | RxJS | Gestisce naturalmente messaggi continui |
| Esecuzione parallela di più elaborazioni asincrone | Promise (`Promise.all`) | Promise sufficiente per semplice esecuzione parallela |
| Stream continui di eventi | RxJS | Può gestire valori multipli in serie temporali |
| Elaborazione cancellabile | RxJS | Cancellazione affidabile con unsubscribe() |
| Applicazione semplice | Promise | Basso costo di apprendimento, poche dipendenze |
| Applicazione Angular | RxJS | Integrato standard nel framework |

### Principi base
- **Usa Promise** se si può mantenere semplice
- **Usa RxJS** se serve elaborazione complessa di stream
- **Combinare entrambi** è anche efficace (ponte con `from()`)

RxJS è potente, ma non è necessario usare RxJS per tutte le elaborazioni asincrone. È importante usare lo strumento giusto nella situazione giusta.

> [!TIP] Prossimi passi
> - Impara i dettagli di Observable in [Cos'è Observable](/it/guide/observables/what-is-observable)
> - Impara come creare Observable in [Creation Functions](/it/guide/creation-functions/index)
> - Impara trasformazione e controllo di Observable in [Operatori](/it/guide/operators/index)
