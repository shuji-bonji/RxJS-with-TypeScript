---
description: "Spiegazione delle Creation Functions per combinare più Observable in uno solo. Imparare le differenze e l'uso di concat, merge, combineLatest, zip, forkJoin e race, i casi d'uso di ciascuna funzione e pattern di implementazione type-safe con TypeScript attraverso esempi di codice pratici."
---

# Creation Functions di combinazione

Le principali Creation Functions per combinare più Observable in un singolo Observable.

## Cosa sono le Creation Functions di combinazione

Le Creation Functions di combinazione ricevono più Observable e li uniscono in un singolo stream Observable. Il timing e l'ordine di emissione dei valori variano a seconda del metodo di combinazione.

Consultare la tabella seguente per le caratteristiche e l'uso di ciascuna Creation Function.

## Principali Creation Functions di combinazione

| Funzione | Descrizione | Casi d'uso |
|----------|------|-------------|
| **[concat](/it/guide/creation-functions/combination/concat)** | Combinazione sequenziale (successivo inizia dopo completamento precedente) | Elaborazione passo-passo |
| **[merge](/it/guide/creation-functions/combination/merge)** | Combinazione parallela (sottoscrizione simultanea, output in ordine di emissione) | Integrazione di più eventi |
| **[combineLatest](/it/guide/creation-functions/combination/combineLatest)** | Combina valori più recenti | Sincronizzazione input form |
| **[zip](/it/guide/creation-functions/combination/zip)** | Accoppia valori corrispondenti | Corrispondenza richiesta-risposta |
| **[forkJoin](/it/guide/creation-functions/combination/forkJoin)** | Attende completamento di tutti e combina valori finali | Attesa completamento chiamate API parallele |

## Criteri di scelta

La selezione delle Creation Functions di combinazione si basa sui seguenti aspetti.

### 1. Timing di esecuzione

- **Esecuzione sequenziale**: `concat` - Inizia il successivo dopo il completamento del precedente Observable
- **Esecuzione parallela**: `merge`, `combineLatest`, `zip`, `forkJoin` - Sottoscrive tutti gli Observable simultaneamente

### 2. Metodo di emissione valori

- **Emette tutti i valori**: `concat`, `merge` - Output di tutti i valori emessi da ciascun Observable
- **Combina valori più recenti**: `combineLatest` - Ogni volta che uno emette un valore, combina ed emette tutti i valori più recenti
- **Accoppia valori corrispondenti**: `zip` - Accoppia ed emette valori di posizioni corrispondenti di ciascun Observable
- **Solo valori finali**: `forkJoin` - Quando tutti gli Observable sono completati, emette un array con i valori finali di ciascuno

### 3. Timing di completamento

- **Dopo completamento di tutti**: `concat`, `forkJoin` - Attende il completamento di tutti gli Observable
- **Completa con stream più breve**: `zip` - Completa quando uno è completato, poiché i restanti valori non possono essere accoppiati
- **Non completa**: `merge`, `combineLatest` - Se uno completa ma altri continuano, non completa

## Conversione da Cold a Hot

Come mostrato nella tabella sopra, **tutte le Creation Functions di combinazione generano Observable Cold**. Un'esecuzione indipendente inizia ad ogni sottoscrizione.

Tuttavia, utilizzando operatori di multicasting (`share()`, `shareReplay()`, `publish()` ecc.), è possibile **convertire Observable Cold in Hot**.

### Esempio pratico: Condivisione richieste HTTP

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Richiesta HTTP indipendente per ogni sottoscrizione
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
);

coldApi$.subscribe(val => console.log('Sottoscrittore 1:', val));
coldApi$.subscribe(val => console.log('Sottoscrittore 2:', val));
// → Ogni sottoscrittore esegue interval indipendente (richieste doppie)

// 🔥 Hot - Condivide esecuzione tra sottoscrittori
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Sottoscrittore 1:', val));
hotApi$.subscribe(val => console.log('Sottoscrittore 2:', val));
// → Condivide un singolo interval (richiesta solo una volta)
```

> [!TIP]
> **Casi in cui è necessario Hot**:
> - Condividere risultati stessa API tra più componenti
> - Utilizzare risultati richieste parallele con `forkJoin` in più punti
> - Gestire stato con `combineLatest` e distribuire a più sottoscrittori
>
> Per dettagli vedere [Creazione base - Conversione da Cold a Hot](/it/guide/creation-functions/basic/#conversione-da-cold-a-hot).

## Corrispondenza con Pipeable Operator

Per le Creation Functions di combinazione esistono Pipeable Operator corrispondenti. Quando si utilizzano all'interno di una pipeline, usare operatori serie `~With`.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Prossimi passi

Per apprendere il funzionamento dettagliato e esempi pratici di ciascuna Creation Function, fare clic sui link nella tabella sopra.

Inoltre, studiando anche [Creation Functions di selezione/partizione](/it/guide/creation-functions/selection/) e [Creation Functions di branching condizionale](/it/guide/creation-functions/conditional/), è possibile comprendere il quadro generale delle Creation Functions.
