---
description: Questa sezione descrive le Funzioni di Creazione che selezionano e creano Observable in base a condizioni. Scopri come usare iif e defer, oltre a esempi pratici.
---

# Funzioni di Creazione Condizionali

Le Funzioni di Creazione selezionano un Observable in base a una condizione o generano dinamicamente un Observable al momento della subscription.

## Cosa Sono le Funzioni di Creazione Condizionali?

Le Funzioni di Creazione Condizionali hanno i seguenti ruoli:

- **Selezione Condizionale**: Seleziona Observable diversi in base alle condizioni
- **Generazione Ritardata**: Crea dinamicamente un Observable alla subscription

A differenza di altre Funzioni di Creazione, che creano e combinano Observable staticamente, queste possono cambiare comportamento in base a **condizioni e stati a runtime**.

> [!NOTE]
> Sebbene `iif` e `defer` fossero precedentemente classificati come "operatori condizionali", sono **Funzioni di Creazione** (funzioni di creazione Observable), non Pipeable Operator.

## Principali Funzioni di Creazione Condizionali

| Funzione | Descrizione | Casi d'Uso |
|----------|------|-------------|
| **[iif](/it/guide/creation-functions/conditional/iif)** | Seleziona uno di due Observable in base a una condizione | Ramificazione elaborazione in base a stato login |
| **[defer](/it/guide/creation-functions/conditional/defer)** | Ritarda generazione dell'Observable al momento della subscription | Creazione dinamica Observable |

## Criteri di Utilizzo

### iif - Due Rami in Base a Condizione

`iif` seleziona uno di due Observable in base al risultato di una funzione condizionale. La condizione viene valutata **al momento della subscription**.

**Sintassi**:
```typescript
iif(
  () => condition,  // Funzione condizione (valutata alla subscription)
  trueObservable,   // Observable se true
  falseObservable   // Observable se false
)
```

**Casi d'Uso**:
- Ramificazione elaborazione in base a stato login
- Cambio elaborazione in base a esistenza cache
- Cambio comportamento per variabili ambiente

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Dati autenticati'),
  of('Dati pubblici')
);

data$.subscribe(console.log);
// Output: 'Dati autenticati' o 'Dati pubblici' (dipende dalla condizione alla subscription)
```

### defer - Generazione Ritardata alla Subscription

`defer` genera un Observable ogni volta che avviene una subscription. Questo permette all'Observable di cambiare comportamento in base al suo stato al momento della subscription.

**Sintassi**:
```typescript
defer(() => {
  // Eseguito al momento della subscription
  return someObservable;
})
```

**Casi d'Uso**:
- Generare Observable che riflette l'ultimo stato al momento della subscription
- Generare un valore casuale diverso ogni volta
- Eseguire elaborazione diversa per ogni subscription

```typescript
import { defer, of } from 'rxjs';

// Ottieni l'ora corrente alla subscription
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Prima:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Seconda:', time));
}, 2000);

// Output:
// Prima: 2024-10-21T01:00:00.000Z
// Seconda: 2024-10-21T01:00:01.000Z
// ※Orari diversi perché i momenti di subscription sono diversi
```

## Differenza tra iif e defer

| Caratteristica | iif | defer |
|------|-----|-------|
| **Scelta** | Seleziona da due Observable | Genera qualsiasi Observable |
| **Timing Valutazione** | Valuta condizione alla subscription | Esegue funzione alla subscription |
| **Scopo** | Ramificazione condizionale | Generazione dinamica |

## Uso in Pipeline

Le Funzioni di Creazione Condizionali possono essere usate in combinazione con altri operatori.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Ottieni informazioni utente da ID utente
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Controlla ultima cache alla subscription
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Conversione da Cold a Hot

Come mostrato nella tabella sopra, **tutte le Funzioni di Creazione Condizionali generano Cold Observable**. Le valutazioni condizionali e le funzioni di generazione vengono eseguite ogni volta che viene fatta una subscription.

Puoi convertire un Cold Observable in Hot Observable usando operatori multicast (`share()`, `shareReplay()`, ecc.).

### Esempio Pratico: Condivisione Risultati Ramificazione Condizionale

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Rivaluta condizione per ogni subscription
const coldIif$ = iif(
  condition,
  of('Condizione è vera'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Subscriber 1:', val));
coldIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Ogni subscriber valuta indipendentemente la condizione (possibilità di risultati diversi)

// 🔥 Hot - Condividi risultati valutazione condizione tra subscriber
const hotIif$ = iif(
  condition,
  of('Condizione è vera'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Subscriber 1:', val));
hotIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Condizione valutata solo una volta, risultati condivisi
```

> [!TIP]
> Per maggiori informazioni, vedi [Creazione Base - Conversione da Cold a Hot](/it/guide/creation-functions/basic/#conversione-da-cold-a-hot).

## Prossimi Passi

Per apprendere il comportamento dettagliato e gli esempi pratici di ogni Funzione di Creazione, clicca sui link dalla tabella sopra.

Inoltre, imparando le [Funzioni di Creazione Combinazione](/it/guide/creation-functions/combination/) e le [Funzioni di Creazione Selezione/Partizione](/it/guide/creation-functions/selection/), puoi comprendere il quadro completo delle Funzioni di Creazione.
