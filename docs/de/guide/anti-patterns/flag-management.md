---
title: Problem der Flaggen-Überschwemmung im Zustandsmanagement
description: "Wie man das Problem von 17 Boolean-Flaggen in RxJS-Projekten durch reaktives Design verbessert. Praktische Refactoring-Methoden zur drastischen Verbesserung von Wartbarkeit und Lesbarkeit, indem Zustände als Observables dargestellt und mehrere Flaggen mit scan() und combineLatest() konsolidiert werden."
# outline: deep
---

# Flaggen-Überschwemmung im Zustandsmanagement

Selbst in Projekten, die RxJS eingeführt haben, ist das Problem der Überschwemmung mit vielen Boolean-Flaggen in Komponenten häufig zu beobachten. Dieser Artikel erklärt anhand eines realen Falls mit 17 Flaggen die Ursachen und Verbesserungsmethoden.

## Praktisches Beispiel des Problems

Schauen wir uns zunächst Code aus der Praxis an. Dies ist ein typisches Beispiel für eine Überschwemmung mit Zustandsverwaltungs-Flaggen:

```typescript
class ProblematicComponent {
  // 17 Flaggen vorhanden
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    // Komplexe Verzweigungen innerhalb von subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Tatsächliche Verarbeitung
              this.isSaving = false;
              this.hasUnsavedChanges = false;
            }
          }
        }
      },
      error: (err) => {
        this.isSaving = false;
        this.hasError = true;
        this.isProcessing = false;
      }
    });
  }
}
```

Solcher Code tritt **auch bei Einführung von RxJS** auf. Dieses Pattern mit manueller Verwaltung von 17 Flaggen und Kontrolle durch komplexe Bedingungsverzweigungen hat Probleme in Bezug auf Wartbarkeit, Lesbarkeit und Testbarkeit.

## Warum kommt es zur Flaggen-Überschwemmung?

Hinter der Flaggen-Überschwemmung stehen nicht nur technische Probleme, sondern auch Denkmuster von Entwicklern und der Entwicklungsprozess von Organisationen. Im Folgenden werden fünf Hauptursachen analysiert.

### Strukturanalyse der Ursachen

| Ursachen-Kategorie | Konkrete Symptome | Hintergrund |
|------------|------------|------|
| **① Verbleib imperativen Denkens** | Mehr als 10 Flaggen wie `isLoading`, `isSaving`, `isError`<br>Viele Guards wie `if (this.isSaving) return;` | Logik wird durch **imperative "Zustandsflaggen"-Kontrolle** statt RxJS-Streams verzweigt.<br>Zustand und Nebeneffekte können nicht getrennt werden, Lesbarkeit sinkt |
| **② Nicht-Nutzung abgeleiteter Zustände** | Direkte Zuweisung wie `this.isLoaded = true;` auf Komponentenseite | Observables `map` und `combineLatest` könnten genutzt werden, um Zustandsableitungen deklarativ zu definieren,<br>aber stattdessen wird Zustand manuell zusammengesetzt |
| **③ Unklare Zuständigkeiten im Zustandsdesign** | Mehrere Flaggen für denselben Zustand<br>(z.B. `isLoadingStart`, `isLoadingEnd`) | **Zustandsänderungen werden als Befehle behandelt**.<br>Was als "ein Zustand" konsolidiert werden sollte, ist auf mehrere Flaggen verteilt |
| **④ Ungeordnete RxJS-Stream-Verzweigungen** | Mehrere `if`s und `tap`s verketten sich in einem `Observable`,<br>Nebeneffekte und Zustandsaktualisierungen vermischen sich | Verantwortungstrennung im Stream-Design ist nicht umgesetzt.<br>Verwendung von `switchMap` und `catchError` ist unklar |
| **⑤ Fehlen einer ViewModel-Schicht** | Direkte Manipulation von `this.isEditing`, `this.isSaved` in UI-Komponenten | Durch das Halten von Zustand in Komponenten<br>werden die Vorteile von RxJS abgeschnitten |


## Grundursache: Inkongruenz der Denkmodelle

Die Grundursache der Flaggen-Überschwemmung ist die **Inkongruenz zwischen imperativer und reaktiver Programmierung**. Wenn Entwickler RxJS mit imperativem Denken verwenden, treten folgende Probleme auf:

### Struktur der Übergangsphase

In vielen Projekten tritt die Flaggen-Hölle durch folgenden Entwicklungsprozess auf:

```
1. Zunächst mit if-Flaggen Kontrolle hinzufügen, um es zum Laufen zu bringen
   ↓
2. Später RxJS einführen
   ↓
3. Alte Logik kann nicht in Streams umgewandelt werden und vermischt sich
   ↓
4. Flaggen-Hölle ist fertig
```

### Vermischung von Zustandsverwaltungsschichten

Zustände in Anwendungen sollten eigentlich in drei Schichten verwaltet werden:

```
Anwendung
 ├── View-Zustand (isOpen, isLoading, formDirty)     ← Innerhalb der Komponente
 ├── Business-Zustand (entity, filters, errors)      ← Zustandsverwaltungsschicht
 └── API-Zustand (pending, success, error)           ← RxJS stream
```

Wenn diese drei Schichten nicht getrennt sind, vermischen sich **drei verschiedene "Flaggen"-Arten** mit unterschiedlichen Zuständigkeiten. Die Verwaltung von View-Zustand und API-Zustand auf derselben Ebene führt zu explosionsartiger Komplexität.

## Wesen des Problems: "Natur" der Flaggen

Das wahre Problem der Flaggen-Überschwemmung ist nicht "viele Flaggen", sondern dass **Flaggen zu imperativen mutable Variablen werden**. Im Folgenden wird der Unterschied zwischen problematischen und geeigneten Flaggen verglichen.

### ❌ Problematische Flaggen: Imperative mutable Variablen

```typescript
class BadComponent {
  // Diese werden zu "Befehlen" statt "Zuständen"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Guard-Klausel erforderlich
    this.isSaving = true;              // Manuelle Änderung

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Manuelles Zurücksetzen
        this.hasError = false;         // Andere Flaggen auch manuell verwalten
      },
      error: () => {
        this.isSaving = false;         // Gleiche Verarbeitung an mehreren Stellen
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problempunkte
> - Zustand ist "prozedural" statt "deklarativ"
> - Timing von Zustandsänderungen ist verstreut
> - Konsistenz zwischen Flaggen muss manuell gewährleistet werden

### ✅ Geeignete Flaggen: Reaktive Variablen

```typescript
class GoodComponent {
  // Als Zustandsstream deklarieren
  private saveAction$ = new Subject<void>();

  readonly saveState$ = this.saveAction$.pipe(
    switchMap(() =>
      this.api.save().pipe(
        map(() => 'success' as const),
        catchError(() => of('error' as const)),
        startWith('loading' as const)
      )
    ),
    startWith('idle' as const),
    shareReplay(1)
  );

  // Abgeleitete Zustände auch deklarativ definieren
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Nur Event auslösen
  }
}
```

> [!TIP] Verbesserungspunkte
> - Zustand wird als "Stream" zentral verwaltet
> - Zustandsübergänge werden deklarativ in Pipeline definiert
> - Konsistenz zwischen Flaggen wird automatisch gewährleistet


## Entscheidungskriterien für Flaggen-Design

Hier sind Kriterien zur Beurteilung, ob Ihr Code problematisches Flaggen-Design aufweist. Nutzen Sie diese als Referenz für Code-Reviews und Design-Entscheidungen.

| Aspekt | ❌ Problematisch | ✅ Unproblematisch |
|------|-----------|-----------|
| **Typ** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **Änderungsmethode** | Direkte Zuweisung `flag = true` | Stream/Ableitung `state$.pipe(map(...))` |
| **Abhängigkeiten** | Implizit (Code-Reihenfolge) | Explizit (combineLatest, computed) |
| **Benennung** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Anzahl** | 10+ unabhängige Booleans | 1 Zustand + mehrere Ableitungen |


## Verbesserungsstrategie

Um das Problem der Flaggen-Überschwemmung zu lösen, gehen Sie schrittweise mit den folgenden drei Schritten vor.

### Schritt 1: Zustandsinventur

Listen Sie zunächst alle aktuellen Flaggen auf und klassifizieren Sie sie nach Zuständigkeiten. Dadurch wird sichtbar, welche Flaggen konsolidiert werden können.

```typescript
// Vorhandene Flaggen auflisten und nach Zuständigkeiten klassifizieren
interface StateInventory {
  view: string[];      // UI-Anzeigesteuerung (isModalOpen, isEditing)
  business: string[];  // Geschäftslogik (isFormValid, hasUnsavedChanges)
  api: string[];       // Kommunikationszustand (isLoading, isSaving, hasError)
}
```

### Schritt 2: Zustand in Enum umwandeln

Als nächstes konsolidieren Sie mehrere verwandte Boolean-Flaggen zu einem Zustand. Zum Beispiel können `isLoading`, `isSaving`, `hasError` alle als "Request-Zustand" konsolidiert werden.

```typescript
// Mehrere Booleans zu einem Zustand konsolidieren
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Verwendungsbeispiel
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError werden unnötig
}
```

### Schritt 3: Reaktivierung

Schließlich verwalten Sie den Zustand mit Observable oder Signal und definieren abgeleitete Zustände deklarativ. Dadurch wird die Konsistenz des Zustands automatisch gewährleistet.

```typescript
// Mit Observable oder Signal verwalten
class ReactiveComponent {
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  // Als ViewModel konsolidieren
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$
  ]).pipe(
    map(([api, form]) => ({
      canSave: !api.saving && form.valid,
      showSpinner: api.loading || api.saving,
      showError: api.error !== null
    }))
  );
}
```


## Implementierungsbeispiel: Refactoring von 17 Flaggen

Hier wird der tatsächliche Refactoring-Prozess der eingangs vorgestellten Komponente mit 17 Flaggen zu reaktivem Design gezeigt. Durch den Before/After-Vergleich können Sie die Verbesserungseffekte erleben.

### Before: Imperative Flaggenverwaltung

Schauen wir uns zunächst den problematischen Code noch einmal an. 17 Boolean-Flaggen überschwemmen und werden durch komplexe Bedingungsverzweigungen kontrolliert.

```typescript
class LegacyComponent {
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    if (!this.isLoading &&
        !this.isSaving &&
        this.isFormValid &&
        !this.hasError &&
        this.isDataLoaded) {
      this.isSaving = true;
      this.apiService.save().subscribe({
        next: () => {
          this.isSaving = false;
          this.hasUnsavedChanges = false;
        },
        error: () => {
          this.isSaving = false;
          this.hasError = true;
        }
      });
    }
  }
}
```

### After: Reaktive Zustandsverwaltung

Schauen wir uns nun den verbesserten Code an. 17 Flaggen wurden in drei Basiszustände (apiState$, formState$, dataState$) und einen abgeleiteten Zustand (vm$) organisiert.

```typescript
import { BehaviorSubject, combineLatest, EMPTY } from 'rxjs';
import { map, switchMap, catchError, startWith } from 'rxjs';

interface ApiState {
  loading: boolean;
  saving: boolean;
  deleting: boolean;
  error: string | null;
}

interface DataState {
  loaded: boolean;
  editing: boolean;
}

class RefactoredComponent {
  // Basiszustände mit Observable verwalten
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    deleting: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  private readonly dataState$ = new BehaviorSubject<DataState>({
    loaded: false,
    editing: false
  });

  // Als ViewModel konsolidieren (abgeleiteter Zustand)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // Abgeleitete Zustände für UI-Anzeige
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Bei Bedarf auch einzelne Zustände veröffentlichen
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // Zustandsprüfung erfolgt automatisch durch ViewModel
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError(error => {
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: error.message
        });
        return EMPTY;
      })
    ).subscribe(() => {
      this.apiState$.next({
        ...this.apiState$.value,
        saving: false
      });
    });
  }
}
```

### Verwendung auf UI-Seite

Durch reaktive Zustandsverwaltung wird auch die Verwendung auf UI-Seite erheblich vereinfacht. Es ist nicht mehr nötig, mehrere Flaggen einzeln zu prüfen, sondern nur noch die erforderlichen Informationen aus dem ViewModel abzurufen.

```typescript
// Before: Direkte Referenz auf mehrere Flaggen
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After: Abgeleiteten Zustand aus ViewModel erhalten
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Wichtigkeit von Benennungsregeln

Bei Flaggen-Design ist die Benennung sehr wichtig. Durch geeignete Benennung können Zuständigkeit, Natur und Lebenszyklus der Flagge auf einen Blick verstanden werden. Umgekehrt ist mehrdeutige Benennung die Quelle von Verwirrung.

### ❌ Schlechte Benennungsbeispiele

Folgende Benennungen sind unklar in der Absicht und verringern die Wartbarkeit.

```typescript
// Was für eine Flagge? Was löst Änderung aus?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// Ist es Zustand? Ist es Aktion?
isProcess: boolean;  // In Bearbeitung? Bearbeitet?
```

### ✅ Gute Benennungsbeispiele

Geeignete Benennung drückt Absicht und Natur des Zustands klar aus. Verwendung von Observable (`$` Suffix) oder Signal, und klare Angabe der Zustandsart (State, can, should).

```typescript
// Zustand klar ausdrücken
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// Abgeleiteter Zustand mit klarer Absicht
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Beispiel mit Signal (verfügbar in Angular, Preact, Solid.js etc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Diagnose-Checkliste

Überprüfen Sie mit der folgenden Checkliste, ob Ihr Code vom Problem der Flaggen-Überschwemmung betroffen ist. Nutzen Sie diese als Referenz für Code-Reviews und Design-Entscheidungen.

```markdown
## 🚨 Warnsignale

- [ ] Es gibt 5 oder mehr Boolean-Variablen
- [ ] 3 oder mehr `if`-Statements sind innerhalb von `subscribe` verschachtelt
- [ ] Dieselbe Flagge wird an mehreren Stellen gesetzt
- [ ] Es gibt 3 oder mehr Benennungen wie `isXXXing`
- [ ] Zustand wird in Komponenten gehalten, obwohl es eine Zustandsverwaltungsschicht gibt
- [ ] Es gibt mehrere Benennungen wie `xxxFlag`
- [ ] Fehlerbehandlung ist über mehrere `subscribe` verstreut

## ✅ Zeichen der Verbesserung

- [ ] Zustand wird mit `Observable` oder `Signal` verwaltet
- [ ] Abgeleitete Zustände werden mit `map`/`computed` definiert
- [ ] Zustandsübergänge werden deklarativ beschrieben
- [ ] ViewModel-Pattern wird angewendet
- [ ] Benennung drückt Absicht klar aus
```

## Zusammenfassung

In diesem Artikel wurden die Ursachen und Verbesserungsmethoden des Problems der Flaggen-Überschwemmung in RxJS-Projekten erklärt. Lassen Sie uns abschließend die wichtigen Punkte zusammenfassen.

### Wesen des Problems

1. **17 Flaggen haben** ← Dies ist ein Symptom
2. **Sie sind imperative mutable Variablen** ← Dies ist das Wesen
3. **Zustandsübergänge sind nicht deklarativ** ← Dies ist die Ursache
4. **Benennung ist mehrdeutig (xxxFlag)** ← Dies ist die Quelle der Verwirrung

### Verbesserungsrichtung

Um das Problem der Flaggen-Überschwemmung zu lösen, sind folgende vier Umstellungen erforderlich:

- **Boolean-Variablen** → **Observable/Signal**
- **Direkte Zuweisung** → **Stream-Pipeline**
- **17 unabhängige** → **1 Zustand + abgeleitete Zustände**
- **xxxFlag** → **xxxState$ / canXXX$**

### Das Wichtigste

> [!IMPORTANT] Wichtiges Prinzip
> "Zustand ist das Ergebnis von Events, nicht direkt durch Flaggen kontrolliert"

Die Einführung von RxJS ist keine "Syntax"-, sondern eine "Philosophie"-Umstellung. Wenn imperatives Denken mitgeschleppt wird, wird die Flaggen-Hölle nicht beseitigt. Durch das Erfassen von Zustand als Stream und deklaratives Design verbessern sich Wartbarkeit, Lesbarkeit und Testbarkeit.


## Verwandte Abschnitte

Um das in diesem Artikel gelernte Wissen über Flaggenverwaltung zu vertiefen, konsultieren Sie bitte auch folgende verwandte Artikel.

- [if-Statement-Verschachtelungs-Hölle innerhalb von subscribe](./subscribe-if-hell) - Geeignete Verarbeitung von Bedingungsverzweigungen
- [Häufige Fehler und Gegenmaßnahmen](./common-mistakes) - Details zu 15 Anti-Patterns
- [Fehlerbehandlung](/de/guide/error-handling/strategies) - Geeignete Fehlerbehandlungsstrategien
- [Subject und Multicasting](/de/guide/subjects/what-is-subject) - Grundlagen der Zustandsverwaltung

## Referenzressourcen

Sie können mit der offiziellen RxJS-Dokumentation und Lernressourcen noch tiefer lernen.

- [RxJS Offizielle Dokumentation](https://rxjs.dev/) - Offizielle API-Referenz und Leitfaden
- [Learn RxJS](https://www.learnrxjs.io/) - Praktische Beispiele nach Operatoren
- [RxJS Marbles](https://rxmarbles.com/) - Visuelles Verständnis des Operator-Verhaltens
