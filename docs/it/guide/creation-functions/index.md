---
description: "Le Creation Functions di RxJS (funzioni di creazione di Observable) spiegate sistematicamente: differenze con i Pipeable Operator, utilizzo di base e 7 categorie (creazione di base, generazione loop, comunicazione HTTP, combinazione, selezione/partizione, condizionale, controllo). Fornisce guida per la selezione in base alle caratteristiche e agli usi di ciascuna funzione."
---

# Creation Functions

In RxJS esistono due forme distinte: le **Creation Functions** per creare Observable e i **Pipeable Operators** per trasformare Observable esistenti.

Questa pagina spiega i concetti di base delle Creation Functions e le 7 categorie principali.

## Cosa sono le Creation Functions

Le **Creation Functions** sono funzioni per creare nuovi Observable.

```typescript
import { of, from, interval } from 'rxjs';

// Utilizzo come Creation Function
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Queste vengono importate direttamente dal pacchetto `rxjs` e chiamate come funzioni per generare Observable.

## Differenza con i Pipeable Operator

Le Creation Functions e i Pipeable Operators differiscono per scopo e utilizzo. Verificare le differenze tra i due nella tabella seguente.

| Caratteristica | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **Scopo** | Creare un nuovo Observable | Trasformare un Observable esistente |
| **Importazione da** | `rxjs` | `rxjs/operators` |
| **Metodo di utilizzo** | Chiamata diretta come funzione | Utilizzo all'interno di `.pipe()` |
| **Esempio** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Esempio di Creation Function

Le Creation Functions vengono utilizzate per combinare direttamente più Observable.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Utilizzo come Creation Function
concat(obs1$, obs2$).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

### Esempio di Pipeable Operator

I Pipeable Operators vengono utilizzati per aggiungere elaborazioni di trasformazione a un Observable esistente.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Utilizzo come Pipeable Operator
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

## Criteri di scelta

La scelta tra Creation Function e Pipeable Operator si basa sui seguenti criteri.

### Quando utilizzare una Creation Function

Le Creation Functions sono adatte quando si operano più Observable allo stesso livello o quando si crea un Observable da zero.

- **Quando si combinano più Observable allo stesso livello**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Quando si crea un Observable da zero**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Quando utilizzare un Pipeable Operator

I Pipeable Operators sono adatti quando si aggiunge un'elaborazione a un Observable esistente o quando si concatenano più operazioni.

- **Quando si aggiunge un'elaborazione a un Observable esistente**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Quando si concatenano più operazioni come pipeline**

## Categorie delle Creation Functions

In questo capitolo, le Creation Functions sono divise in 7 categorie per l'apprendimento.

### Elenco di tutte le categorie

Nella tabella seguente è possibile verificare tutte le categorie e le funzioni incluse. Facendo clic sul nome di ciascuna funzione si passa alla pagina dei dettagli.

| Categoria | Descrizione | Funzioni principali | Casi d'uso rappresentativi |
|---------|------|-----------|-------------------|
| **[Creazione di base](/it/guide/creation-functions/basic/)** | Funzioni più basilari e frequentemente utilizzate. Creano Observable basati su dati, array, eventi e tempo | [of](/it/guide/creation-functions/basic/of), [from](/it/guide/creation-functions/basic/from), [fromEvent](/it/guide/creation-functions/basic/fromEvent), [interval](/it/guide/creation-functions/basic/interval), [timer](/it/guide/creation-functions/basic/timer) | Test di valori fissi, streaming di dati esistenti, gestione eventi DOM, polling, esecuzione ritardata |
| **[Generazione loop](/it/guide/creation-functions/loop/)** | Esprime elaborazioni loop come for e while con Observable | [range](/it/guide/creation-functions/loop/range), [generate](/it/guide/creation-functions/loop/generate) | Generazione di numeri sequenziali, elaborazione batch, transizioni di stato complesse, calcoli matematici |
| **[Comunicazione HTTP](/it/guide/creation-functions/http-communication/)** | Gestisce la comunicazione HTTP come Observable | [ajax](/it/guide/creation-functions/http-communication/ajax), [fromFetch](/it/guide/creation-functions/http-communication/fromFetch) | Comunicazione HTTP basata su XMLHttpRequest, comunicazione HTTP basata su Fetch API, chiamate REST API |
| **[Combinazione](/it/guide/creation-functions/combination/)** | Combina più Observable in uno. Il timing e l'ordine di emissione differiscono in base al metodo di combinazione | [concat](/it/guide/creation-functions/combination/concat), [merge](/it/guide/creation-functions/combination/merge), [combineLatest](/it/guide/creation-functions/combination/combineLatest), [zip](/it/guide/creation-functions/combination/zip), [forkJoin](/it/guide/creation-functions/combination/forkJoin) | Elaborazione step-by-step, unificazione di eventi multipli, sincronizzazione di input di form, attesa del completamento di chiamate API parallele |
| **[Selezione/Partizione](/it/guide/creation-functions/selection/)** | Seleziona uno da più Observable o divide un Observable in più | [race](/it/guide/creation-functions/selection/race), [partition](/it/guide/creation-functions/selection/partition) | Competizione tra più origini dati, diramazione elaborazione successo/fallimento |
| **[Condizionale](/it/guide/creation-functions/conditional/)** | Seleziona Observable in base a condizioni o genera dinamicamente al momento della sottoscrizione | [iif](/it/guide/creation-functions/conditional/iif), [defer](/it/guide/creation-functions/conditional/defer) | Diramazione elaborazione in base allo stato di login, creazione dinamica di Observable, valutazione lazy |
| **[Controllo](/it/guide/creation-functions/control/)** | Controlla il timing di esecuzione e la gestione delle risorse degli Observable | [scheduled](/it/guide/creation-functions/control/scheduled), [using](/it/guide/creation-functions/control/using) | Controllo timing di esecuzione tramite scheduler, gestione lifecycle risorse, prevenzione memory leak |

> [!TIP]
> **Ordine di apprendimento**
>
> Per i principianti si raccomanda di apprendere nel seguente ordine.
> 1. **Creazione di base** - Funzioni fondamentali di RxJS
> 2. **Combinazione** - Fondamenti per gestire più stream
> 3. **Comunicazione HTTP** - Integrazione API pratica
> 4. Altre categorie - Apprendimento secondo necessità

## Corrispondenza con i Pipeable Operator

Molte Creation Functions hanno Pipeable Operators corrispondenti. Quando si utilizzano all'interno di una pipeline, si usano gli operatori della serie `~With`.

| Creation Function | Pipeable Operator | Note |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/it/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/it/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/it/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/it/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/it/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Da RxJS 7 in poi, sono stati aggiunti operatori della serie `~With` come **[concatWith](/it/guide/operators/combination/concatWith)**, **[mergeWith](/it/guide/operators/combination/mergeWith)**, **[zipWith](/it/guide/operators/combination/zipWith)**, **[combineLatestWith](/it/guide/operators/combination/combineLatestWith)**, **[raceWith](/it/guide/operators/combination/raceWith)**, rendendoli più facili da usare anche come Pipeable Operators.

## Quale utilizzare?

La scelta tra Creation Function e Pipeable Operator dipende dal contesto.

### Creation Function raccomandato

Quando si operano più Observable allo stesso livello, l'utilizzo di una Creation Function rende il codice più conciso.

```typescript
// ✅ Combinazione di più Observable allo stesso livello
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator raccomandato

Quando si aggiunge un'operazione come parte di una pipeline, l'utilizzo di un Pipeable Operator rende chiaro il flusso di elaborazione.

```typescript
// ✅ Combinazione come parte della pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Riepilogo

- **Creation Functions**: Funzioni per creare o combinare Observable
- **Pipeable Operators**: Funzioni per trasformare Observable esistenti
- Le Creation Functions sono classificate in 7 categorie
  1. **Creazione di base**: Crea Observable basati su dati, array, eventi, tempo
  2. **Generazione loop**: Esprime elaborazioni ripetitive con Observable
  3. **Comunicazione HTTP**: Gestisce comunicazione HTTP come Observable
  4. **Combinazione**: Unisce più in uno
  5. **Selezione/Partizione**: Seleziona o divide
  6. **Condizionale**: Genera dinamicamente in base a condizioni
  7. **Controllo**: Controlla timing di esecuzione e gestione risorse
- All'interno delle pipeline si usano i Pipeable Operators della serie `~With`
- Ogni categoria include più funzioni, da usare in base allo scopo

## Prossimi passi

Per apprendere i dettagli di ciascuna categoria, procedere dai seguenti link.

1. **[Creation Functions di Creazione di base](/it/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Creation Functions di Generazione loop](/it/guide/creation-functions/loop/)** - range, generate
3. **[Creation Functions di Comunicazione HTTP](/it/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Creation Functions di Combinazione](/it/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Creation Functions di Selezione/Partizione](/it/guide/creation-functions/selection/)** - race, partition
6. **[Creation Functions Condizionali](/it/guide/creation-functions/conditional/)** - iif, defer
7. **[Creation Functions di Controllo](/it/guide/creation-functions/control/)** - scheduled, using

In ogni pagina è possibile apprendere il funzionamento dettagliato e gli esempi pratici delle Creation Functions.

## Risorse di riferimento

- [Documentazione ufficiale RxJS - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
