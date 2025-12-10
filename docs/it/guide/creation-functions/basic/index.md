---
description: Spiegazione delle Creation Functions per la creazione basilare di Observable. Imparare come creare Observable da varie fonti di dati come valori singoli, array, Promise, eventi e timer utilizzando of, from, fromEvent, interval e timer. Implementabile con type safety di TypeScript, un concetto importante che costituisce la base di RxJS.
---

# Creation Functions di Creazione di base

Le Creation Functions più basilari e frequentemente utilizzate. Creano facilmente Observable basati su dati, array, eventi e tempo.

## Cosa sono le Creation Functions di Creazione di base

Le Creation Functions di creazione di base sono funzioni per creare un singolo Observable da varie fonti di dati. Questi sono il gruppo di funzioni più fondamentale quando si utilizza RxJS e vengono utilizzate in quasi tutto il codice RxJS.

Nella tabella seguente, verificare le caratteristiche e l'utilizzo di ciascuna Creation Function.

## Principali Creation Functions di Creazione di base

| Function | Descrizione | Caso d'uso |
|----------|------|-------------|
| **[of](/it/guide/creation-functions/basic/of)** | Emette i valori specificati in ordine | Test di valori fissi, creazione mock |
| **[from](/it/guide/creation-functions/basic/from)** | Converte da array, Promise, ecc. | Streaming di dati esistenti |
| **[fromEvent](/it/guide/creation-functions/basic/fromEvent)** | Converte eventi in Observable | Eventi DOM, Node.js EventEmitter |
| **[interval](/it/guide/creation-functions/basic/interval)** | Emette continuamente a intervalli specificati | Polling, esecuzione periodica |
| **[timer](/it/guide/creation-functions/basic/timer)** | Inizia l'emissione dopo un ritardo | Esecuzione ritardata, timeout |

## Criteri di scelta

La scelta delle Creation Functions di creazione di base è determinata dal tipo di fonte dati.

### 1. Tipo di dati

- **Valori statici**: `of()` - Crea Observable specificando direttamente i valori
- **Array o iterabili**: `from()` - Converte collezioni esistenti in stream
- **Promise**: `from()` - Converte elaborazioni asincrone in Observable
- **Eventi**: `fromEvent()` - Converte event listener in Observable
- **Basati su tempo**: `interval()`, `timer()` - Emette valori in base al trascorrere del tempo

### 2. Timing di emissione

- **Emissione immediata**: `of()`, `from()` - Inizia l'emissione dei valori immediatamente dopo la sottoscrizione
- **All'occorrenza dell'evento**: `fromEvent()` - Emette ogni volta che si verifica un evento
- **Emissione periodica**: `interval()` - Emissione continua a intervalli costanti
- **Emissione dopo ritardo**: `timer()` - Inizia l'emissione dopo un tempo specificato

### 3. Timing di completamento

- **Completa immediatamente**: `of()`, `from()` - Completa dopo aver emesso tutti i valori
- **Non completa**: `fromEvent()`, `interval()` - Continua fino a unsubscribe
- **Emette una volta e completa**: `timer(delay)` - Completa dopo aver emesso un valore

## Esempi di utilizzo pratico

### of() - Test di valori fissi

```typescript
import { of } from 'rxjs';

// Creare dati di test
const mockUser$ = of({ id: 1, name: 'Test User' });

mockUser$.subscribe(user => console.log(user));
// Output: { id: 1, name: 'Test User' }
```

### from() - Streaming di array

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6, 8, 10
```

### fromEvent() - Evento click

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Button clicked!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Polling API ogni 5 secondi
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Updated:', data));
```

### timer() - Esecuzione ritardata

```typescript
import { timer } from 'rxjs';

// Esegue dopo 3 secondi
timer(3000).subscribe(() => console.log('3 seconds passed'));
```

## Attenzione ai memory leak

Quando si utilizzano le Creation Functions di creazione di base, è importante la corretta cancellazione della sottoscrizione.

> [!WARNING]
> `fromEvent()`, `interval()`, e `timer(delay, period)` che emette periodicamente non completano, quindi è necessario eseguire sempre `unsubscribe()` alla distruzione del componente, o annullare automaticamente con `takeUntil()`, ecc.
>
> Nota: `timer(delay)` senza il secondo argomento completa automaticamente dopo un'emissione.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Window resized'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Conversione da Cold a Hot

Come mostrato nella tabella sopra, **tutte le Creation Functions di creazione di base generano Cold Observable**. Ad ogni sottoscrizione inizia un'esecuzione indipendente.

Tuttavia, utilizzando i seguenti operatori di multicasting, è possibile **convertire Cold Observable in Hot Observable**.

### Condizioni e operatori per Hot-izzare

| Operatore | Comportamento | Caso d'uso |
|-------------|------|-------------|
| **share()** | Multicast + connessione/disconnessione automatica | Condividere richieste HTTP tra più sottoscrittori |
| **shareReplay(n)** | Memorizza gli ultimi n valori e li distribuisce ai nuovi sottoscrittori | Cache delle risposte API |
| **publish() + connect()** | Avvia multicast manualmente | Inizia l'esecuzione quando i sottoscrittori sono pronti |
| **multicast(subject)** | Multicast con Subject personalizzato | Quando è necessario un controllo avanzato |

### Esempio pratico

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Timer indipendente per ogni sottoscrizione
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (dopo 0 secondi)
// A: 1 (dopo 1 secondo)
// B: 0 (dopo 1.5 secondi) ← B inizia indipendentemente da 0
// A: 2 (dopo 2 secondi)
// B: 1 (dopo 2.5 secondi)

// 🔥 Hot - Condivide il timer tra i sottoscrittori
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (dopo 0 secondi)
// A: 1 (dopo 1 secondo)
// A: 2, B: 2 (dopo 2 secondi) ← B si unisce a metà, riceve lo stesso valore
```

> [!TIP]
> **Casi in cui è necessario Hot-izzare**:
> - Si vuole condividere una richiesta HTTP tra più sottoscrittori
> - Si vuole mantenere una sola connessione WebSocket o server
> - Si vuole utilizzare il risultato di un calcolo costoso in più punti
>
> Per maggiori dettagli, consultare il capitolo **Subject e Multicasting** (Chapter 5).

## Relazione con i Pipeable Operator

Le Creation Functions di creazione di base non hanno Pipeable Operators corrispondenti diretti. Queste vengono sempre utilizzate come Creation Functions.

Tuttavia, vengono utilizzate in combinazione con i Pipeable Operators nei seguenti pattern.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Input utente → Attesa 300ms → Chiamata API
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Prossimi passi

Per apprendere il funzionamento dettagliato e gli esempi pratici di ciascuna Creation Function, fare clic sui link nella tabella sopra.

Inoltre, apprendendo anche le [Creation Functions di Combinazione](/it/guide/creation-functions/combination/), le [Creation Functions di Selezione/Partizione](/it/guide/creation-functions/selection/) e le [Creation Functions Condizionali](/it/guide/creation-functions/conditional/), è possibile comprendere il quadro completo delle Creation Functions.
