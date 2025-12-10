---
description: "Una spiegazione dettagliata della sintassi di separazione delle fasi che elimina l'\"inferno one-liner\" di RxJS. Separando chiaramente la definizione dello stream, la trasformazione e la sottoscrizione, e dando un nome a ogni fase, è possibile scrivere codice reattivo più facile da debuggare, testare e leggere. Include esempi pratici di refactoring."
---

# Inferno one-liner e sintassi di separazione delle fasi

Il motivo principale per cui il codice RxJS sembra un "inferno one-liner" è che **"definizioni degli stream", "trasformazioni" e "sottoscrizioni (side effect)" sono mescolate insieme**. Questo riduce significativamente la leggibilità e la facilità di debug.

## Perché si verifica l'"inferno one-liner"?

### ❌ Codice problematico comune

```ts
import { fromEvent } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

fromEvent(document, 'click')
  .pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    debounceTime(300),
    switchMap(x => ajax(`/api?x=${x}`))
  )
  .subscribe(res => {
    if (res.status === 200) {
      console.log('OK');
    } else {
      handleError(res);
    }
  });

function handleError(res: any) {
  console.error('Error:', res);
}
```

### Problemi

| Problema | Impatto |
|---|---|
| **Righe troppo lunghe** | I lettori si perdono |
| **Difficile da debuggare** | Difficile verificare lo stato intermedio |
| **Difficile da testare** | L'unico modo è testare l'intero stream |
| **Struttura annidata** | Le ramificazioni condizionali tendono ad essere profonde in subscribe |
| **Non riutilizzabile** | L'elaborazione della pipeline non può essere usata altrove |


## Soluzione: sintassi di separazione delle fasi (Functional Style)

Organizzare il codice RxJS in una "struttura a tre fasi con relazioni chiare".

1. **Definizione dello stream (source)** - fonte dei dati
2. **Trasformazione dello stream (pipeline)** - elaborazione dei dati
3. **Sottoscrizione e side effect (subscription)** - side effect come aggiornamenti UI e log


## Pattern consigliato: sintassi di separazione delle fasi

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Definizione Observable (fonte dello stream)
const clicks$ = fromEvent(document, 'click');

// 2. Definizione pipeline (elaborazione di trasformazione dei dati)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Elaborazione sottoscrizione (esecuzione dei side effect)
const subscription = processed$.subscribe({
  next: x => console.log('Posizione click:', x),
  error: err => console.error(err),
  complete: () => console.log('Completato')
});
```

### Vantaggi

| Vantaggio | Dettagli |
|---|---|
| **Significato chiaro per ogni step** | Le responsabilità di ogni fase sono evidenti a colpo d'occhio |
| **Facile da debuggare** | Verificare lo stream intermedio con `console.log` o `tap` |
| **Facile da testare** | Testare stream intermedi come `processed$` individualmente |
| **Annidamento ridotto** | Elaborazione semplificata in subscribe |
| **Riutilizzabile** | L'elaborazione della pipeline può essere estratta come funzione |


## Variante: separazione delle funzioni (modularizzazione)

Se il processo di trasformazione è lungo, **separare la pipeline come funzione**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Estrarre l'elaborazione della pipeline come funzione
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Lato utilizzo
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Punto chiave:** Estrarre "come trasformare" come funzione pura **aumenta enormemente la testabilità**.


## Convenzioni di naming

Un naming appropriato chiarisce l'intento del codice.

| Fase | Esempio di naming | Significato |
|---|---|---|
| **Source** | `clicks$`, `input$`, `routeParams$` | Fonte di eventi e dati |
| **Pipe** | `processed$`, `validInput$`, `apiResponse$` | Stream elaborati |
| **Subscription** | `subscription`, `uiSubscription` | Side effect effettivamente eseguiti |

Il suffisso **`$`** indica a colpo d'occhio che si tratta di un Observable.


## Per una scrittura più dichiarativa (RxJS 7+)

Estrarre `pipe` come funzione e renderla riutilizzabile.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Definire la pipeline come funzione (riutilizzabile)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Vantaggio:** La logica di elaborazione (`processClicks`) può essere riutilizzata in altri stream.


## Before/After: refactoring per pattern tipici

Esempi di miglioramenti in casi d'uso reali.

### A. Evento UI → API → Aggiornamento UI

#### ❌ Before (inferno one-liner)

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, switchMap, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

interface ApiRes {
  items: string[];
  error?: string;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

fromEvent(button, 'click').pipe(
  throttleTime(500),
  switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
  catchError(err => of({ items: [], error: err.message }))
).subscribe(res => {
  list.innerHTML = res.items.map(item => `<li>${item}</li>`).join('');
  if (res.error) alert(res.error);
});
```

#### ✅ After (separazione delle fasi + funzionalizzazione)

```ts
import { fromEvent, pipe, of } from 'rxjs';
import { throttleTime, switchMap, map, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface ApiRes {
  items: string[];
}

interface Result {
  items: string[];
  error: string | null;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

// 1) source
const clicks$ = fromEvent(button, 'click');

// 2) pipeline (estratta come funzione pura)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (solo side effect)
const subscription = result$.subscribe(({ items, error }) => {
  renderList(items);
  if (error) toast(error);
});

function renderList(items: string[]) {
  list.innerHTML = items.map(item => `<li>${item}</li>`).join('');
}

function toast(message: string) {
  alert(message);
}
```

**Miglioramenti:**
- Elaborazione pipeline `loadItems()` trasformata in funzione pura
- Side effect (`renderList`, `toast`) consolidati lato subscribe
- Più facile da testare e debuggare


### B. Valori form → validazione → salvataggio API (auto-save)

#### ❌ Before

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

fromEvent(input, 'input')
  .pipe(
    map((e: Event) => (e.target as HTMLInputElement).value),
    debounceTime(400),
    distinctUntilChanged(),
    filter(v => v.length >= 3),
    switchMap(v => ajax.post('/api/save', { v }))
  )
  .subscribe(
    () => console.log('OK'),
    err => alert(err.message)
  );
```

#### ✅ After (separazione delle responsabilità + naming)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (validazione)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (auto-save)
const autosave = () =>
  pipe(
    switchMap((v: string) => ajax.post('/api/save', { v }))
  );

const save$ = value$.pipe(validate(), autosave());

// 3) subscription
const subscription = save$.subscribe({
  next: () => showSuccess(),
  error: (err) => showError(String(err?.message ?? err))
});

function showSuccess() {
  console.log('Salvato');
}

function showError(message: string) {
  alert(message);
}
```

**Miglioramenti:**
- Separazione di validazione (`validate`) e salvataggio (`autosave`)
- Ogni pipeline ora è riutilizzabile
- Test più semplici (validazione e salvataggio possono essere testati separatamente)


### C. Cache + refresh manuale

```ts
import { merge, of, Subject } from 'rxjs';
import { switchMap, shareReplay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Item {
  id: number;
  name: string;
}

const refreshBtn = document.getElementById('refresh-btn') as HTMLButtonElement;

// 1) sources
const refresh$ = new Subject<void>();
const initial$ = of(void 0);

// 2) pipeline
const fetchItems$ = merge(initial$, refresh$).pipe(
  switchMap(() => ajax.getJSON<Item[]>('/api/items')),
  shareReplay({ bufferSize: 1, refCount: true }) // memoizzazione
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// Ricarica da UI
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Punti chiave:**
- Separare auto-load iniziale (`initial$`) e refresh manuale (`refresh$`)
- Cache dei valori più recenti con `shareReplay`
- Più sottoscrittori condividono gli stessi risultati


## Avanzato: incorporare log intermedi

Puoi osservare ogni fase con `tap()`.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Click avvenuto')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('Coordinata X:', x))
);

processed$.subscribe(x => console.log('Valore finale:', x));
```

**Punti chiave:**
- `tap` è un operatore dedicato ai side effect
- Puoi osservare il valore di ogni fase durante il debug
- Dovrebbe essere rimosso in produzione


## Dimostrare la testabilità

La separazione delle fasi consente di **testare l'elaborazione della pipeline in isolamento**.

### Esempio: test della validazione input

```ts
// validate.ts
import { pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs';

export const validateQuery = () =>
  pipe(
    map((s: string) => s.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((s) => s.length >= 3)
  );
```

```ts
// validate.spec.ts
import { TestScheduler } from 'rxjs/testing';
import { validateQuery } from './validate';

describe('validateQuery', () => {
  it('trims, debounces, distincts, filters length>=3', () => {
    const scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });

    scheduler.run(({ hot, expectObservable }) => {
      // Input: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Atteso: passano solo 'abc' e 'abcd'
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Vantaggi:**
- L'elaborazione della pipeline può essere testata **in isolamento**
- Indipendente da DOM/HTTP = **veloce e stabile**
- I marble test controllano la timeline

Per maggiori informazioni, vedere [Metodologie di test](/it/guide/testing/unit-tests).


## Template di istruzioni per GitHub Copilot

Una raccolta di prompt utilizzabili per il refactoring reale.

### 1. Scomposizione in tre livelli

```
Refactoring di questo codice RxJS scomponendolo in una struttura a tre livelli "source / pipeline / subscription".
Requisiti:
- Gli Observable sono denominati con suffisso $
- Le pipeline sono estratte come funzioni che restituiscono pipe(...) (es: validate(), loadItems())
- I side effect (aggiornamenti UI, console, toast) sono consolidati in subscribe
- Mettere tap() nei punti appropriati (con commenti) per osservare lo stato intermedio
- I nomi delle variabili e delle funzioni devono comunicare il dominio
```

### 2. Chiarire la selezione dell'operatore

```
Voglio prevenire chiamate API multiple dovute a click multipli.
Suggerisci quale tra switchMap/mergeMap/concatMap/exhaustMap usare,
e sostituisci con l'operatore corretto. Scrivi le motivazioni nei commenti.

Linee guida:
- Il salvataggio form è sequenziale (concatMap)
- I suggerimenti di ricerca scartano le vecchie richieste (switchMap)
- Nessuna doppia esecuzione per click ripetuti (exhaustMap)
```

### 3. Pattern auto-save

```
Refactoring del seguente codice in pattern auto-save:
- Input con debounceTime e distinctUntilChanged
- Salvataggio serializzato con concatMap
- I side effect per notifiche di successo/fallimento all'UI sono lato subscribe
- Funzionalizzare la trasformazione per facilitare i test
- Se possibile, cache dello stato più recente con shareReplay
```

### 4. Cache + refresh manuale

```
Passare al pattern "primo auto-load + refresh manuale":
- Introdurre refresh$ Subject
- merge(initial$, refresh$) → switchMap(fetch)
- Cache dei valori più recenti con shareReplay({bufferSize:1, refCount:true})
- Estrarre la pipe di fetch come funzione per il riutilizzo
```


## Conclusione: sintesi delle linee guida per una scrittura leggibile

| Voce | Contenuto consigliato |
|---|---|
| ✅ 1 | Observable, pipe e subscribe scritti **separatamente** |
| ✅ 2 | Gli stream intermedi **indicano il significato con i nomi delle variabili** |
| ✅ 3 | Le pipe complesse sono **funzionalizzate** |
| ✅ 4 | **tap() permette verifiche intermedie** |
| ✅ 5 | Riutilizzabile con `processSomething = pipe(...)` |


## Riepilogo

- **L'inferno one-liner** è causato dalla commistione di definizione stream, trasformazione e sottoscrizione
- La **sintassi di separazione delle fasi** (Source → Pipeline → Subscription) chiarisce le responsabilità
- **Funzionalizzare la pipeline** migliora testabilità e riutilizzabilità
- **Il naming appropriato** (suffisso `$`, nomi di variabili significativi) migliora la leggibilità

## Sezioni correlate

- **[Errori comuni e come affrontarli](/it/guide/anti-patterns/common-mistakes#13-eccessiva-complessita)** - Anti-pattern dell'eccessiva complessità
- **[Metodologie di test](/it/guide/testing/unit-tests)** - Come testare il codice RxJS
- **[Comprendere gli operatori](/it/guide/operators/)** - Come usare ogni operatore

## Prossimi passi

1. Cercare le aree del codice esistente che sono nell'"inferno one-liner"
2. Refactoring con la sintassi di separazione delle fasi
3. Funzionalizzare i processi della pipeline e scrivere unit test
4. Usare i template di istruzioni Copilot per unificare tutto il team


> [!NOTE]
> Una trattazione più completa di "Come scrivere RxJS leggibile" sarà affrontata nel futuro **Capitolo 13: Pattern pratici**.
