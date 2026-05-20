---
description: "Vengono illustrate le Creation Function (race e partition) che selezionano uno tra più Observable o dividono un Observable in più Observable. Vengono presentati casi d'uso pratici e implementazioni sicure per TypeScript, come la gestione dei conflitti, l'acquisizione della risposta più rapida e il partizionamento del flusso tramite ramificazione condizionale."
---

# Sistemi di selezione e partizione Creation Function

Creation Function che seleziona un Observable da più Observable o divide un Observable in più Observable in base a condizioni.

## Sistemi di selezione e suddivisione Cosa sono le Creation Function?

Le Creation Function per i sistemi di selezione e divisione sono diverse da quelle per i sistemi di combinazione e hanno i seguenti ruoli.

- **Selezione**: seleziona un Observable che soddisfa determinate condizioni tra più Observable.
- **Splitting**: suddivide un Observable in più Observable in base a una condizione.

Queste funzioni operano nella direzione opposta o da una prospettiva diversa rispetto al join "combina più osservabili in uno".

## Principali sistemi di selezione e divisione Creation Function

| Funzione | Descrizione | Caso d'uso. |
|---|---|---|
| **[race](/it/guide/creation-functions/selezione/race)** | Adotta il primo pubblicato | Gara con più fonti di dati |
| **[partition](/it/guide/creation-functions/selezione/partition)** | Dividere in due con condizioni | Processo di ramificazione per successo/fallimento |

## Criteri di utilizzo

### race - selezionare l'Observable più veloce

race si abbona a diversi Observable contemporaneamente e adotta il **primo Observable che emette un valore. Gli Observable che non vengono adottati vengono automaticamente unsubscribe.

**Caso d'uso**:.
- Adottare la risposta più veloce da più endpoint API.
- Gestione del timeout (processo originale o timer)
- Concorrenza tra cache e chiamate API effettive

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Adottare il più veloce da più fonti di dati
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Uscita.: 'Fast API' (1L'uscita viene emessa dopo un secondo,slow$viene annullato)
```

### partition - suddivisione per condizione

Il metodo `partition` divide un Observable in **due Observable** in base a una funzione condizionale. Il valore di ritorno è un array `[se vero, se falso]`.

**Caso d'uso**:.
- Separazione di successo e fallimento.
- Separazione di numeri pari e dispari.
- Separazione di dati validi e non validi.

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividere in numeri pari e dispari
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Uscita.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Uscita.: Odd: 1, Odd: 3, Odd: 5
```

## Conversione da freddo a caldo

Come indicato nella tabella precedente, **tutte le funzioni di selezione e di creazione di sistemi suddivisi generano un Observable freddo**. Ogni sottoscrizione avvia un'esecuzione indipendente.

Gli Observable freddi possono essere convertiti in Observable caldi utilizzando gli operatori multicast (`share()`, `shareReplay()`, ecc.).

### Esempio pratico: condivisione di richieste API in conflitto tra loro

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Ripetere la gara per ogni abbonamento
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Abbonato1:', val));
coldRace$.subscribe(val => console.log('Abbonato2:', val));
// → Ogni abbonato esegue una gara indipendente (una2competizione una volta)

// 🔥 Hot - Condividere i risultati delle gare tra gli abbonati
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abbonato1:', val));
hotRace$.subscribe(val => console.log('Abbonato2:', val));
// → 1Condividere i risultati di un concorso
```

> [!TIP]

> Per ulteriori informazioni, vedere [Sistema di creazione di base - Conversione da freddo a caldo] (/it/guide/creation-functions/basic/#cold- to -hot-).

## Corrispondenza con Pipeable Operator

Le funzioni di selezione e di divisione Creation Function hanno anche un corrispondente Pipeable Operator.

| Creation Function | Pipeable Operator |
|---|---|
| race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(sorgente$, predicato)` | Non può essere utilizzato all'interno di una pipeline (solo Creation Function). |

> [!NOTE]

> Non esiste una versione Pipeable Operator di `partition`. Se è necessaria una partizione, è possibile utilizzarla come Creation Function o dividerla manualmente utilizzando due volte filter.

## Prossimi passi.

Per saperne di più sul funzionamento dettagliato e sugli esempi pratici di Creation Function, fare clic sui collegamenti della tabella precedente.

È inoltre possibile imparare [Funzioni di creazione combinate](/it/guide/creation-functions/combinazione/) e [Funzioni di creazione condizionali](/it/guide/creation-functions/condizionali/). Insieme, forniscono una comprensione olistica delle Creation Function.
