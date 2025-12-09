---
description: "Auflösung der RxJS-'Einzeiler-Hölle' durch Phasentrennungs-Syntax detailliert erklärt. Durch klare Trennung von Stream-Definition, Transformation und Subscription können Sie reaktiven Code schreiben, der leicht zu debuggen, zu testen und zu lesen ist. Mit praktischen Refactoring-Beispielen."
---

# Einzeiler-Hölle und Phasentrennungs-Syntax

Der Hauptgrund, warum RxJS-Code wie eine "Einzeiler-Hölle" aussieht, ist, dass **"Stream-Definition", "Transformation" und "Subscription (Nebeneffekte)" vermischt sind**. Dies verringert erheblich die Lesbarkeit und Debuggbarkeit.

## Warum entsteht "Einzeiler-Hölle"?

### ❌ Häufiger Problemcode

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

### Problempunkte

| Problem | Auswirkung |
|---|---|
| **Lange Zeilen** | Leser verlieren sich |
| **Schwieriges Debugging** | Zwischenzustände schwer zu überprüfen |
| **Schwieriges Testen** | Kann nur gesamten Stream testen |
| **Verschachtelte Verarbeitungsstruktur** | Bedingungsverzweigungen innerhalb von subscribe werden tief |
| **Nicht wiederverwendbar** | Pipeline-Verarbeitung kann nicht anderswo verwendet werden |


## Lösung: Phasentrennungs-Syntax (Functional Style)

RxJS-Code in "drei Phasen mit klaren Beziehungen" organisieren.

1. **Stream-Definition (source)** - Datenquelle
2. **Stream-Transformation (pipeline)** - Datenverarbeitung
3. **Subscription und Nebeneffekte (subscription)** - Nebeneffekte wie UI-Update und Logging


## Empfohlenes Pattern: Phasentrennungs-Syntax

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Observable-Definition (Quelle des Streams)
const clicks$ = fromEvent(document, 'click');

// 2. Pipeline-Definition (Datentransformationsverarbeitung)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Subscription-Verarbeitung (Ausführung von Nebeneffekten)
const subscription = processed$.subscribe({
  next: x => console.log('Klickposition:', x),
  error: err => console.error(err),
  complete: () => console.log('Abgeschlossen')
});
```

### Vorteile

| Vorteil | Details |
|---|---|
| **Bedeutung jedes Schritts ist klar** | Zuständigkeit jeder Phase auf einen Blick erkennbar |
| **Einfaches Debugging** | Zwischenstreams können mit `console.log` oder `tap` überprüft werden |
| **Einfaches Testen** | Zwischenstreams wie `processed$` können einzeln getestet werden |
| **Flache Verschachtelung** | Verarbeitung innerhalb von subscribe bleibt einfach |
| **Wiederverwendbar** | Pipeline-Verarbeitung kann als Funktion extrahiert werden |


## Variation: Funktionstrennung (Modularisierung)

Wenn Transformationsverarbeitung lang wird, **Pipeline als Funktion trennen**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Pipeline-Verarbeitung als Funktion extrahieren
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Verwendungsseite
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Wichtiger Punkt:** Wenn "wie transformiert wird" als reine Funktion extrahiert wird, **explodiert die Testbarkeit**.


## Benennungsregeln (Naming Rule)

Absicht des Codes durch geeignete Benennung klar machen.

| Phase | Benennungsbeispiel | Bedeutung |
|---|---|---|
| **Quelle** | `clicks$`, `input$`, `routeParams$` | Event- oder Datenquelle |
| **Pipeline** | `processed$`, `validInput$`, `apiResponse$` | Verarbeiteter Stream |
| **Subscription** | `subscription`, `uiSubscription` | Tatsächlich ausgeführte Nebeneffekte |

**`$` Suffix** macht auf einen Blick klar, dass es sich um ein Observable handelt.


## Deklarativeres Schreiben (ab RxJS 7)

`pipe` als Funktion extrahieren und wiederverwendbar machen.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Pipeline als Funktion definieren (wiederverwendbar)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Vorteil:** Verarbeitungslogik (`processClicks`) kann auch in anderen Streams wiederverwendet werden.


## Before/After: Typisches Pattern-Refactoring

Verbesserungsbeispiele in tatsächlichen Anwendungsfällen.

### A. UI-Event → API → UI-Update

#### ❌ Before (Einzeiler-Hölle)

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

#### ✅ After (Phasentrennung + Funktionalisierung)

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

// 2) pipeline (als reine Funktion extrahiert)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (nur Nebeneffekte)
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

**Verbesserungspunkte:**
- Pipeline-Verarbeitung `loadItems()` als reine Funktion
- Nebeneffekte (`renderList`, `toast`) auf subscribe-Seite konsolidiert
- Einfacher zu testen und zu debuggen


### B. Formularwert → Validierung → API-Speicherung (Autosave)

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

#### ✅ After (Zuständigkeitstrennung + Benennung)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (Validierung)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (Autosave)
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
  console.log('Gespeichert');
}

function showError(message: string) {
  alert(message);
}
```

**Verbesserungspunkte:**
- Validierung (`validate`) und Speicherung (`autosave`) getrennt
- Jede Pipeline ist wiederverwendbar
- Einfaches Testen (Validierung und Speicherung können einzeln getestet werden)


### C. Cache + Manueller Refresh

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
  shareReplay({ bufferSize: 1, refCount: true }) // Memoisierung
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// UI-Neuladung
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Wichtiger Punkt:**
- Initiales automatisches Laden (`initial$`) und manueller Refresh (`refresh$`) getrennt
- Neuester Wert mit `shareReplay` gecacht
- Mehrere Abonnenten teilen sich dasselbe Ergebnis


## Fortgeschritten: Einbetten von Zwischenlogs

Mit `tap()` können Sie jede Phase beobachten.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Klick aufgetreten')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('X-Koordinate:', x))
);

processed$.subscribe(x => console.log('Endwert:', x));
```

**Wichtiger Punkt:**
- `tap` ist ein Operator nur für Nebeneffekte
- Ermöglicht Überprüfung der Werte jeder Phase beim Debugging
- Sollte in Produktionsumgebung entfernt werden


## Nachweis der Testbarkeit

Durch Phasentrennung kann **Pipeline-Verarbeitung einzeln getestet** werden.

### Beispiel: Test der Eingabevalidierung

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
      // Eingabe: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Erwartung: Nur 'abc' und 'abcd' passieren
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Vorteile:**
- Pipeline-Verarbeitung kann **einzeln** getestet werden
- Keine Abhängigkeit von DOM/HTTP = **schnell und stabil**
- Zeitachse mit Marble-Tests kontrolliert

Details siehe [Testmethoden](/de/guide/testing/unit-tests).


## GitHub Copilot Anweisungsvorlagen

Prompt-Sammlung für tatsächliches Refactoring.

### 1. Zerlegung in drei Phasen

```
Refactoring dieses RxJS-Codes in "source / pipeline / subscription" drei Phasen.
Anforderungen:
- Observables mit $ Suffix benennen
- Pipeline als pipe(...) zurückgebende Funktion extrahieren (z.B.: validate(), loadItems())
- Nebeneffekte (UI-Update, console, toast) in subscribe konsolidieren
- Zwischenzustände mit tap() beobachtbar machen (mit Kommentar)
- Variablen- und Funktionsnamen sollen Domäne vermitteln
```

### 2. Klarstellung der Operator-Auswahl

```
Möchte mehrfache API-Aufrufe durch häufige Klicks verhindern.
Bitte vorschlagen, welcher von den aktuellen switchMap/mergeMap/concatMap/exhaustMap verwendet werden sollte,
und durch richtigen Operator ersetzen. Begründung als Kommentar schreiben.

Richtlinien:
- Formularspeicherung ist sequentielle Verarbeitung (concatMap)
- Suchvorschläge brechen alte Requests ab (switchMap)
- Button-Mehrfachklick verhindert doppelte Ausführung (exhaustMap)
```

### 3. Autosave-Pattern

```
Refactoring des folgenden Codes zu Autosave-Pattern:
- Eingabe mit debounceTime und distinctUntilChanged
- Speicherung mit concatMap serialisieren
- Nebeneffekte für Erfolg/Fehler auf subscribe-Seite konzentrieren
- Transformation funktionalisieren für Testbarkeit
- Falls möglich, neuesten Zustand mit shareReplay cachen
```

### 4. Cache + Manueller Refresh

```
Zu "initiales automatisches Laden + manueller Refresh" Pattern ändern:
- refresh$ Subject einführen
- merge(initial$, refresh$) → switchMap(fetch)
- Neuesten Wert mit shareReplay({bufferSize:1, refCount:true}) cachen
- Fetch-Pipeline als Funktion extrahieren für Wiederverwendbarkeit
```


## Fazit: Richtlinien für lesbares Schreiben

| Punkt | Empfohlener Inhalt |
|---|---|
| ✅ 1 | Observable, pipe und subscribe **getrennt schreiben** |
| ✅ 2 | Zwischenstreams mit **Variablennamen Bedeutung geben** |
| ✅ 3 | Komplexe pipes **funktionalisieren** |
| ✅ 4 | **tap() für Zwischenüberprüfung** ermöglichen |
| ✅ 5 | `processSomething = pipe(...)` wiederverwendbar machen |


## Zusammenfassung

- **Einzeiler-Hölle** entsteht durch Vermischung von Stream-Definition, Transformation und Subscription
- **Phasentrennungs-Syntax** (Source → Pipeline → Subscription) macht Zuständigkeiten klar
- **Funktionalisierung von Pipelines** verbessert Testbarkeit und Wiederverwendbarkeit
- **Geeignete Benennung** (`$` Suffix, aussagekräftige Variablennamen) verbessert Lesbarkeit

## Verwandte Abschnitte

- **[Häufige Fehler und Gegenmaßnahmen](/de/guide/anti-patterns/common-mistakes#13-übermäßige-komplexität)** - Anti-Pattern übermäßiger Komplexität
- **[Testmethoden](/de/guide/testing/unit-tests)** - Testmethoden für RxJS-Code
- **[Verständnis von Operatoren](/de/guide/operators/)** - Verwendung jedes Operators


## Nächste Schritte

1. Stellen, die zur "Einzeiler-Hölle" geworden sind, im vorhandenen Code finden
2. Mit Phasentrennungs-Syntax refactoren
3. Pipeline-Verarbeitung funktionalisieren und Unit-Tests schreiben
4. Copilot-Anweisungsvorlagen nutzen und im gesamten Team vereinheitlichen


> [!NOTE]
> Ein umfassenderes "Lesbares Schreiben von RxJS" wird voraussichtlich in **Kapitel 12: Praktische Patterns** behandelt.
