---
description: Questa sezione fornisce una panoramica delle Funzioni di Creazione di Selezione e Partizione che selezionano un Observable da più Observable o dividono un Observable in più Observable. Spiega come usare race e partition, oltre a esempi pratici.
---

# Funzioni di Creazione Selezione/Partizione

Queste sono Funzioni di Creazione per selezionare un Observable da più Observable o dividere un Observable in più Observable.

## Cosa Sono le Funzioni di Creazione Selezione/Partizione?

Le Funzioni di Creazione Selezione/Partizione sono un insieme di funzioni che competono tra più Observable per selezionare il più veloce, o dividono un Observable in due stream basandosi su condizioni. Questo è utile per competere tra sorgenti dati o allocare l'elaborazione in base a condizioni.

Controlla la tabella sotto per vedere le caratteristiche e l'utilizzo di ogni Funzione di Creazione.

## Principali Funzioni di Creazione Selezione/Partizione

| Funzione | Descrizione | Casi d'Uso |
|----------|------|-------------|
| **[race](/it/guide/creation-functions/selection/race)** | Seleziona l'Observable più veloce (quello che emette per primo) | Competizione tra più sorgenti dati, elaborazione fallback |
| **[partition](/it/guide/creation-functions/selection/partition)** | Dividi in due Observable basandosi su una condizione | Gestione successo/fallimento, ramificazione basata su condizioni |

## Criteri di Utilizzo

La selezione delle Funzioni di Creazione Selezione/Partizione è determinata dalle seguenti prospettive.

### 1. Scopo

- **Seleziona il più veloce da più sorgenti**: `race` - Seleziona il primo che emette un valore tra più sorgenti dati
- **Dividi per condizione**: `partition` - Dividi un Observable in due stream basandosi su una condizione

### 2. Timing di Emissione

- **Solo il più veloce**: `race` - Una volta selezionato, gli altri valori Observable vengono ignorati
- **Classifica tutti i valori**: `partition` - Tutti i valori vengono ordinati in due stream secondo le condizioni

### 3. Timing di Completamento

- **Dipende dall'Observable selezionato**: `race` - Segue il completamento dell'Observable che ha emesso per primo
- **Dipende dall'Observable originale**: `partition` - Entrambi gli stream completano quando l'Observable originale completa

## Esempi di Utilizzo Pratico

### race() - Seleziona il Più Veloce da Più Sorgenti Dati

Se hai più sorgenti dati e vuoi usare quella che risponde più velocemente, usa `race()`.

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Simula più API
const api1$ = timer(1000).pipe(map(() => 'Risposta API1'));
const api2$ = timer(500).pipe(map(() => 'Risposta API2'));
const api3$ = timer(1500).pipe(map(() => 'Risposta API3'));

// Usa la risposta più veloce
race(api1$, api2$, api3$).subscribe(console.log);
// Output: Risposta API2 (più veloce a 500ms)
```

### partition() - Dividi in Due Basandosi su Condizione

Se vuoi dividere un Observable in due stream basandoti su una condizione, usa `partition()`.

```typescript
import { of } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// Dividi in numeri pari e dispari
const [evens$, odds$] = partition(numbers$, n => n % 2 === 0);

evens$.subscribe(n => console.log('Pari:', n));
// Output: Pari: 2, Pari: 4, Pari: 6, Pari: 8, Pari: 10

odds$.subscribe(n => console.log('Dispari:', n));
// Output: Dispari: 1, Dispari: 3, Dispari: 5, Dispari: 7, Dispari: 9
```

## Conversione da Cold a Hot

Come mostrato nella tabella sopra, **tutte le Funzioni di Creazione Selezione/Partizione generano Cold Observable**. Viene avviata un'esecuzione indipendente per ogni subscription.

Tuttavia, usando operatori multicast (`share()`, `shareReplay()`, ecc.), puoi **convertire un Cold Observable in Hot Observable**.

### Esempio Pratico: Condivisione dell'Esecuzione

```typescript
import { race, timer, share } from 'rxjs';
import { map } from 'rxjs';

// ❄️ Cold - Esecuzione indipendente per ogni subscription
const coldRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
);

coldRace$.subscribe(val => console.log('Subscriber 1:', val));
coldRace$.subscribe(val => console.log('Subscriber 2:', val));
// → Ogni subscriber esegue race indipendente (2x richieste)

// 🔥 Hot - Condividi esecuzione tra subscriber
const hotRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
).pipe(share());

hotRace$.subscribe(val => console.log('Subscriber 1:', val));
hotRace$.subscribe(val => console.log('Subscriber 2:', val));
// → Condividi esecuzione race (richiesta solo una volta)
```

> [!TIP]
> **Casi in cui serve conversione Hot**:
> - Condividere il risultato di `race()` tra più componenti
> - Usare il risultato di `partition()` in più posizioni
> - Eseguire elaborazione ad alto costo solo una volta
>
> Per maggiori informazioni, vedi [Creazione Base - Conversione da Cold a Hot](/it/guide/creation-functions/basic/#conversione-da-cold-a-hot).

## Corrispondenza con Pipeable Operator

Per le Funzioni di Creazione Selezione/Partizione, c'è un Pipeable Operator corrispondente. Quando usato in una pipeline, si usa l'operatore di tipo `~With`.

| Funzione di Creazione | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Nessuna corrispondenza diretta (usa come Funzione di Creazione) |

> [!NOTE]
> `partition()` è tipicamente usato come Funzione di Creazione. Per eseguire la divisione dello stream all'interno di una pipeline, usa operatori come `filter()` in combinazione.

## Prossimi Passi

Per apprendere il comportamento dettagliato e gli esempi pratici di ogni Funzione di Creazione, clicca sui link dalla tabella sopra.

Inoltre, imparando le [Funzioni di Creazione Base](/it/guide/creation-functions/basic/), le [Funzioni di Creazione Combinazione](/it/guide/creation-functions/combination/) e le [Funzioni di Creazione Condizionali](/it/guide/creation-functions/conditional/), puoi comprendere il quadro completo delle Funzioni di Creazione.
