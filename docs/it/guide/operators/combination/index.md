---
description: "Gli operatori di combinazione di RxJS (Pipeable Operators) per combinare più Observable. Spieghiamo withLatestFrom, concatWith, mergeWith, zipWith, raceWith, le differenze con le Creation Functions e una guida alla selezione per caso d'uso."
---

# Operatori di Combinazione (Pipeable Operators)

Gli operatori di combinazione (Combination) di RxJS sono strumenti potenti per combinare più Observable e creare nuovi stream.

> [!IMPORTANT]
> Questa pagina tratta i **Pipeable Operators (formato utilizzato all'interno delle pipeline)**.
>
> Per le **Creation Functions (formato che crea nuovi Observable da più Observable)**,
> consultare il [Capitolo 3 Creation Functions](/it/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Le funzionalità relative alla combinazione sono fornite in due formati.

### Creation Functions (spiegate nel Capitolo 3)

Accettano più Observable come argomenti e creano un nuovo Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Utilizzate come Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

Per i dettagli, consultare [Creation Functions](/it/guide/creation-functions/).

### Pipeable Operators (spiegati in questa pagina)

Utilizzati all'interno di `.pipe()` su Observable esistenti.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Utilizzati come Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Elenco dei Pipeable Operators

### ◾ Operatori trattati in questa pagina

|Operatore|Descrizione|
|---|---|
|[withLatestFrom](./withLatestFrom)|Combina con i valori più recenti di altri stream in risposta alle emissioni dell'Observable principale|
|[mergeAll](./mergeAll)|Appiattisce un Higher-order Observable in parallelo|
|[concatAll](./concatAll)|Appiattisce un Higher-order Observable in sequenza|
|[switchAll](./switchAll)|Passa all'Higher-order Observable più recente|
|[exhaustAll](./exhaustAll)|Ignora nuovi Higher-order Observable durante l'esecuzione|
|[combineLatestAll](./combineLatestAll)|Combina i valori più recenti di tutti gli Observable interni|
|[zipAll](./zipAll)|Accoppia i valori corrispondenti di ciascun Observable interno|

### ◾ Forniti come Creation Functions

I seguenti sono principalmente utilizzati come Creation Functions (consultare il [Capitolo 3](/it/guide/creation-functions/)).

|Function|Descrizione|Versione Pipeable|
|---|---|---|
|[concat](/it/guide/creation-functions/combination/concat)|Combinazione sequenziale|`concatWith` (RxJS 7+)|
|[merge](/it/guide/creation-functions/combination/merge)|Combinazione parallela|`mergeWith` (RxJS 7+)|
|[combineLatest](/it/guide/creation-functions/combination/combineLatest)|Combina valori più recenti|`combineLatestWith` (RxJS 7+)|
|[zip](/it/guide/creation-functions/combination/zip)|Accoppia valori corrispondenti|`zipWith` (RxJS 7+)|
|[race](/it/guide/creation-functions/selection/race)|Adotta lo stream più veloce|`raceWith` (RxJS 7+)|
|[forkJoin](/it/guide/creation-functions/combination/forkJoin)|Attende il completamento di tutti|(Nessuna versione Pipeable)|

## Per approfondire con esempi pratici

Esempi di scenari reali utilizzando gli operatori di combinazione sono presentati in dettaglio in
[Casi d'uso pratici](./practical-use-cases.md).
