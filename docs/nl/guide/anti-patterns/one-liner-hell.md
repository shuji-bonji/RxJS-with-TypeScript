---
description: "Gedetailleerde uitleg over het oplossen van RxJS 'one-liner hell' met fase-scheidings syntax. Door stream definitie, transformatie en subscribe duidelijk te scheiden en elke fase een naam te geven, kun je reactieve code schrijven die gemakkelijk te debuggen, testen en lezen is. Met praktische refactoring-voorbeelden."
---

# One-liner Hell en Fase-scheidings Syntax

De belangrijkste reden waarom RxJS code eruitziet als "one-liner hell" is dat **"stream definitie", "transformatie" en "subscribe (bijwerkingen)" door elkaar lopen**. Dit vermindert leesbaarheid en debugbaarheid aanzienlijk.

## Waarom Ontstaat "One-liner Hell"

### ❌ Veelvoorkomende Probleemcode

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
  console.error('Fout:', res);
}
```

### Problemen

| Probleem | Impact |
|---|---|
| **Lange regels** | Lezers raken verdwaald |
| **Moeilijk te debuggen** | Moeilijk om tussenliggende status te controleren |
| **Moeilijk te testen** | Alleen hele stream testen mogelijk |
| **Geneste verwerkingsstructuur** | Voorwaardelijke vertakkingen binnen subscribe worden diep |
| **Niet herbruikbaar** | Pipeline verwerking niet bruikbaar elders |


## Oplossing: Fase-scheidings Syntax (Functional Style)

Organiseer RxJS code in een "3-fasen structuur met duidelijke relaties".

1. **Stream definitie (source)** - Databron
2. **Stream transformatie (pipeline)** - Dataverwerking
3. **Subscribe en bijwerkingen (subscription)** - Bijwerkingen zoals UI-updates of logs


## Aanbevolen Patroon: Fase-scheidings Syntax

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Observable definitie (streambron)
const clicks$ = fromEvent(document, 'click');

// 2. Pipeline definitie (datatransformatie verwerking)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Subscribe verwerking (bijwerkingen uitvoeren)
const subscription = processed$.subscribe({
  next: x => console.log('Klikpositie:', x),
  error: err => console.error(err),
  complete: () => console.log('Voltooid')
});
```

### Voordelen

| Voordeel | Details |
|---|---|
| **Betekenis duidelijk per stap** | Verantwoordelijkheid van elke fase in één oogopslag duidelijk |
| **Gemakkelijk te debuggen** | Tussenliggende streams controleren met `console.log` of `tap` |
| **Gemakkelijk te testen** | Tussenliggende streams zoals `processed$` individueel testen |
| **Ondiepe nesting** | Verwerking binnen subscribe wordt eenvoudig |
| **Herbruikbaar** | Pipeline verwerking kan als functie worden geëxtraheerd |


## Variatie: Functie Scheiding (Modularisatie)

Als transformatieverwerking lang wordt, **scheiden we de pipeline als functie**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Extraheer pipeline verwerking als functie
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Gebruik kant
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Punt:** Als je "hoe te transformeren" als pure functie extraheert, **explodeert testbaarheid**.


## Naamgevingsregels (Naming Rule)

Maak de intentie van code duidelijk met juiste naamgeving.

| Fase | Naamgevingsvoorbeeld | Betekenis |
|---|---|---|
| **Source** | `clicks$`, `input$`, `routeParams$` | Bron van events of data |
| **Pipe** | `processed$`, `validInput$`, `apiResponse$` | Verwerkte stream |
| **Subscription** | `subscription`, `uiSubscription` | Daadwerkelijk uitgevoerde bijwerking |

Door **`$` suffix** toe te voegen is "het is een Observable" in één oogopslag duidelijk.


## Meer Declaratief Schrijven (RxJS 7 en later)

Extraheer `pipe` als functie en maak het herbruikbaar.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Definieer pipeline als functie (herbruikbaar)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Voordeel:** Verwerkingslogica (`processClicks`) ook herbruikbaar in andere streams.


## Before/After: Refactoring per Typisch Patroon

We introduceren verbeteringsvoorbeelden in daadwerkelijke use cases.

### A. UI Event → API → UI Update

#### ❌ Before (One-liner Hell)

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

#### ✅ After (Fase Scheiding + Functie-isering)

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

// 2) pipeline (extraheren naar pure functie)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (alleen bijwerkingen)
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

**Verbeterpunten:**
- Pipeline verwerking `loadItems()` als pure functie
- Bijwerkingen (`renderList`, `toast`) geaggregeerd aan subscribe kant
- Gemakkelijk te testen en debuggen


### B. Formulierwaarde → Validatie → API Opslaan (Auto-opslaan)

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

#### ✅ After (Verantwoordelijkheid Scheiding + Naamgeving)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (validatie)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (auto-opslaan)
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
  console.log('Opgeslagen');
}

function showError(message: string) {
  alert(message);
}
```

**Verbeterpunten:**
- Validatie (`validate`) en opslaan (`autosave`) gescheiden
- Elke pipeline herbruikbaar
- Gemakkelijk te testen (validatie en opslaan individueel testbaar)


### C. Cache + Handmatig Verversen

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
  shareReplay({ bufferSize: 1, refCount: true }) // Memoïsatie
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// Herladen vanuit UI
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Punt:**
- Initiële automatische laadactie (`initial$`) en handmatig verversen (`refresh$`) gescheiden
- Cache laatste waarde met `shareReplay`
- Meerdere abonnees delen hetzelfde resultaat


## Gevorderd: Tussenliggende Logs Inbedden

Met `tap()` kun je elke fase observeren.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Klik opgetreden')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('X coördinaat:', x))
);

processed$.subscribe(x => console.log('Eindwaarde:', x));
```

**Punt:**
- `tap` is operator voor bijwerkingen
- Bij debuggen kun je waarde van elke fase controleren
- Moet worden verwijderd in productieomgeving


## Bewijs van Testbaarheid

Door fase-scheiding kun je **pipeline verwerking individueel testen**.

### Voorbeeld: Invoer Validatie Test

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
      // Invoer: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Verwachting: alleen 'abc' en 'abcd' passeren
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Voordelen:**
- Pipeline verwerking **individueel** testen
- Niet afhankelijk van DOM/HTTP = **snel & stabiel**
- Tijdas controleren met marble test

Zie [Testmethoden](/nl/guide/testing/unit-tests) voor details.


## GitHub Copilot Instructie Sjabloon

Verzameling prompts bruikbaar bij daadwerkelijke refactoring.

### 1. Opsplitsing in Drie Fasen

```
Refactor deze RxJS code door op te splitsen in "source / pipeline / subscription" 3 fasen.
Vereisten:
- Observable benoemen met $ suffix
- Pipeline extraheren als functie die pipe(...) retourneert (bijv: validate(), loadItems())
- Bijwerkingen (UI updates, console, toast) aggregeren binnen subscribe
- tap() invoegen op geschikte plaatsen om tussenliggende status te kunnen observeren (met commentaar)
- Variabele en functienamen zodanig dat domein overdraagbaar is
```

### 2. Verduidelijking Operator Selectie

```
Wil voorkomen dat meerdere API calls door herhaalde klikken.
Stel voor welke van de huidige switchMap/mergeMap/concatMap/exhaustMap moet worden gebruikt,
en vervang door juiste operator. Schrijf rationale in commentaar.

Richtlijnen:
- Formulier opslaan is sequentiële verwerking (concatMap)
- Zoeksuggesties verwerpen oude requests (switchMap)
- Button spamming verbiedt dubbele uitvoering (exhaustMap)
```

### 3. Auto-opslaan Patroon

```
Refactor onderstaande code naar auto-opslaan patroon:
- Invoer met debounceTime en distinctUntilChanged
- Opslaan serialiseren met concatMap
- Bijwerkingen om succes/falen naar UI te melden naar subscribe kant
- Transformatie als functie voor testbaarheid
- Indien mogelijk laatste status cachen met shareReplay
```

### 4. Cache + Handmatig Verversen

```
Wijzig naar "initiële automatische laadactie + handmatig verversen" patroon:
- Introduceer refresh$ Subject
- merge(initial$, refresh$) → switchMap(fetch)
- Cache laatste waarde met shareReplay({bufferSize:1, refCount:true})
- Extraheer fetch pipe als functie voor hergebruik
```


## Conclusie: Richtlijnen voor Leesbaar Schrijven Samengevat

| Item | Aanbevolen Inhoud |
|---|---|
| ✅ 1 | Observable・pipe・subscribe **gescheiden schrijven** |
| ✅ 2 | Tussenliggende streams **betekenis tonen met variabelenaam** |
| ✅ 3 | Complexe pipe **als functie** |
| ✅ 4 | **Tussenliggende controle met tap()** mogelijk maken |
| ✅ 5 | Herbruikbaar maken met `processSomething = pipe(...)` |


## Samenvatting

- **One-liner hell** ontstaat door vermenging van stream definitie, transformatie en subscribe
- **Fase-scheidings syntax** (Source → Pipeline → Subscription) verduidelijkt verantwoordelijkheden
- **Pipeline als functie** verbetert testbaarheid en herbruikbaarheid
- **Juiste naamgeving** (`$` suffix, betekenisvolle variabelenamen) verbetert leesbaarheid

## Gerelateerde Gedeelten

- **[Veelvoorkomende Fouten en Oplossingen](/nl/guide/anti-patterns/common-mistakes#13-overmatige-complexiteit)** - Overmatige complexiteit anti-patroon
- **[Testmethoden](/nl/guide/testing/unit-tests)** - Hoe RxJS code te testen
- **[Begrip van Operators](/nl/guide/operators/)** - Hoe elke operator te gebruiken

## Volgende Stappen

1. Zoek plaatsen in bestaande code waar "one-liner hell" is
2. Refactor met fase-scheidings syntax
3. Maak pipeline verwerking als functie en schrijf unit tests
4. Gebruik Copilot instructie sjabloon om te unificeren over heel team


> [!NOTE]
> Meer alomvattende "hoe leesbare RxJS te schrijven" wordt behandeld in toekomstig **Chapter 12: Praktische Patronen**.
