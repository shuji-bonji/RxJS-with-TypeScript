---
description: Descrive come implementare il multicasting usando l'operatore share(). Descrive come condividere lo stesso Observable con più subscriber per ridurre l'elaborazione duplicata e fornisce opzioni di controllo dettagliate.
titleTemplate: ':title | RxJS'
---

# share - Condividi un Observable con Più Subscriber

L'operatore `share()` è l'operatore di multicasting più semplice da implementare in RxJS.
Più subscriber possono condividere la stessa fonte dati per ridurre l'elaborazione duplicata (richieste API, elaborazione computazionale, ecc.).

[📘 Documentazione Ufficiale RxJS - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Utilizzo Base

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable che conta a intervalli
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Sorgente: ${value}`)),
  share() // Abilita multicasting
);

// Primo subscriber
console.log('Sottoscrizione Observer 1 iniziata');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// Aggiungi secondo subscriber dopo 2.5 secondi
setTimeout(() => {
  console.log('Sottoscrizione Observer 2 iniziata');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // Cancella sottoscrizione subscriber 1 dopo 2.5 secondi
  setTimeout(() => {
    console.log('Observer 1 cancellato');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Risultato Esecuzione

```
Sottoscrizione Observer 1 iniziata
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Sottoscrizione Observer 2 iniziata
Sorgente: 2
Observer 1: 2
Observer 2: 2
Sorgente: 3
Observer 1: 3
Observer 2: 3
Observer 1 cancellato
Sorgente: 4
Observer 2: 4
```

**Punti Importanti**:
- L'elaborazione sorgente (`tap`) viene eseguita solo una volta
- Tutti i subscriber ricevono lo stesso valore
- I subscriber che si uniscono a metà riceveranno solo i valori dopo l'unione

## 💡 Come Funziona share()

`share()` è un operatore di multicasting standard di RxJS. Internamente, usa Subject per trasmettere a più subscriber.

> [!NOTE]
> **Modifiche in RxJS v7 e successivi**: Precedentemente spiegato come combinazione di `multicast()` e `refCount()`, questi operatori sono stati deprecati in v7 e rimossi in v8. Attualmente, `share()` è il metodo di multicasting standard. Per dettagli, vedi [Documentazione Ufficiale RxJS - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Flusso di Operazione**:
- **Alla prima sottoscrizione**: Avvia una connessione all'Observable sorgente e crea un Subject interno
- **Aggiungi subscriber**: Condividi la connessione esistente (trasmetti valori attraverso Subject)
- **Tutti i subscriber cancellati**: Disconnetti dalla sorgente (se `resetOnRefCountZero: true`)
- **Risottoscrizione**: Inizia come nuova connessione (a seconda dell'impostazione di reset)

## 🎯 Opzioni di Controllo Avanzato (RxJS 7+)

In RxJS 7 e successivi, puoi passare opzioni a `share()` per controllare finemente il suo comportamento.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Sorgente: ${value}`)),
  share({
    resetOnError: true,       // Reset in caso di errore
    resetOnComplete: true,     // Reset al completamento
    resetOnRefCountZero: true, // Reset quando il conteggio subscriber raggiunge zero
  })
);
```

### Opzioni in Dettaglio

| Opzione | Default | Descrizione |
|-----------|----------|------|
| `resetOnError` | `true` | Resetta lo stato interno in caso di errore |
| `resetOnComplete` | `true` | Resetta lo stato interno al completamento dello stream |
| `resetOnRefCountZero` | `true` | Disconnetti quando il conteggio subscriber raggiunge zero |
| `connector` | `() => new Subject()` | Specifica Subject personalizzato |

### Controllo Avanzato Usando l'Opzione connector

Usando l'opzione `connector`, puoi ottenere un comportamento equivalente a `shareReplay`.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Buffer dell'ultimo 1 elemento usando ReplaySubject
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Sorgente: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Primo subscriber
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Sottoscrivi dopo 2.5 secondi (riceve l'ultimo 1 elemento)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**Risultato Esecuzione**:
```
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Observer 2: 1  // ← Riceve il valore precedente anche unendosi a metà
Sorgente: 2
Observer 1: 2
Observer 2: 2
...
```

> [!TIP]
> Questo metodo può essere usato come alternativa a `shareReplay(1)`. Impostando `resetOnRefCountZero: false`, puoi mantenere la connessione anche quando il conteggio riferimenti raggiunge zero, evitando il problema della "cache persistente" di `shareReplay`.

## 📊 Confronto con e senza share()

### ❌ Senza share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Sorgente: ${value}`))
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Risultato Esecuzione**:
```
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Sorgente: 0    // ← Inizia nuovo stream
Observer 2: 0
Sorgente: 2
Observer 1: 2
Sorgente: 1
Observer 2: 1
Sorgente: 2
Observer 2: 2
```

Ogni subscriber ha uno stream indipendente, e l'elaborazione sorgente viene eseguita in modo ridondante.

### ✅ Con share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Sorgente: ${value}`)),
  share()
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Risultato Esecuzione**:
```
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Observer 2: 1  // ← Condivide lo stesso stream
Sorgente: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Casi d'Uso Pratici

### Prevenzione Richieste API Duplicate

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable per ottenere informazioni utente
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('Richiesta API eseguita')),
  share() // Previeni richieste duplicate in più componenti
);

// Componente 1
getUser$.subscribe(user => console.log('Componente 1:', user));

// Componente 2 (richiede quasi simultaneamente)
getUser$.subscribe(user => console.log('Componente 2:', user));

// Risultato: La richiesta API viene eseguita solo una volta
```

### Condivisione Recupero Dati Periodico

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Ottieni lista TODO ogni 5 secondi (condividi richiesta API)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('Richiesta API eseguita')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // Condividi richiesta API tra più subscriber
);

// Usa lo stesso stream dati in più componenti
sharedTodos$.subscribe(todos => console.log('Componente A:', todos));
sharedTodos$.subscribe(todos => console.log('Componente B:', todos));

// Risultato: La richiesta API viene eseguita solo una volta ogni 5 secondi, entrambi i componenti ricevono gli stessi dati
```

## ⚠️ Note Importanti

1. **Attenzione al timing**: I subscriber che si uniscono a metà non possono ricevere valori passati
2. **Propagazione errori**: Quando si verifica un errore, tutti i subscriber sono interessati
3. **Gestione memoria**: Non cancellare correttamente le sottoscrizioni può causare memory leak

## 🔄 Operatori Correlati

- **[shareReplay()](/it/guide/operators/multicasting/shareReplay)** - Bufferizza valori passati e li fornisce ai subscriber successivi
- **[Subject](/it/guide/subjects/what-is-subject)** - La classe che forma la base del multicasting

> [!WARNING]
> **Operatori deprecati**: Vecchie API di multicasting come `publish()`, `multicast()`, `refCount()` sono state deprecate in RxJS v7 e rimosse in v8. Usa `share()` o `connectable()`/`connect()` invece.

## Riepilogo

L'operatore `share()`:
- Condivide lo stesso Observable tra più subscriber
- Previene l'esecuzione duplicata di richieste API ed elaborazioni pesanti
- Base del multicasting facile da usare
- Opzioni di controllo granulare disponibili in RxJS 7+

Quando più componenti necessitano della stessa fonte dati, usare `share()` può migliorare significativamente le performance.
