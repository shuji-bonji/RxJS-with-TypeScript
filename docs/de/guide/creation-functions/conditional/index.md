---
description: "Bedingte Creation Functions: iif für Verzweigung zwischen zwei Observables, defer für verzögerte Evaluierung bei Subscription. TypeScript-Beispiele."
---

# Bedingte Creation Functions

Creation Functions, die Observables basierend auf Bedingungen auswählen oder zum Zeitpunkt der Subscription dynamisch generieren.

## Was sind bedingte Creation Functions?

Bedingte Creation Functions haben folgende Aufgaben:

- **Auswahl nach Bedingung**: Auswahl unterschiedlicher Observables je nach Bedingung
- **Verzögerte Generierung**: Dynamische Generierung von Observables zum Zeitpunkt der Subscription

Im Gegensatz zu anderen Creation Functions, die Observables statisch erstellen oder kombinieren, können diese ihr Verhalten basierend auf **Laufzeitbedingungen oder -zuständen** ändern.

> [!NOTE]
> `iif` und `defer` wurden früher als "bedingte Operatoren" klassifiziert, sind jedoch **Creation Functions** (Observable-Erstellungsfunktionen) und keine Pipeable Operators.

## Hauptsächliche bedingte Creation Functions

| Function | Beschreibung | Anwendungsfall |
|----------|--------------|----------------|
| **[iif](/de/guide/creation-functions/conditional/iif)** | Wählt zwischen zwei Observables basierend auf einer Bedingung | Verzweigung basierend auf Login-Status |
| **[defer](/de/guide/creation-functions/conditional/defer)** | Verzögerte Generierung eines Observables bei Subscription | Dynamische Observable-Erstellung |

## Auswahlkriterien

### iif - Zwei Verzweigungen basierend auf Bedingung

`iif` wählt eines von zwei Observables basierend auf dem Ergebnis einer Bedingungsfunktion. Die Bedingung wird **zum Zeitpunkt der Subscription** ausgewertet.

**Syntax**:
```typescript
iif(
  () => condition,  // Bedingungsfunktion (wird bei Subscription ausgewertet)
  trueObservable,   // Observable für true-Fall
  falseObservable   // Observable für false-Fall
)
```

**Anwendungsfälle**:
- Verarbeitungsverzweigung je nach Login-Status
- Verarbeitungswechsel je nach Cache-Verfügbarkeit
- Verhaltensänderung basierend auf Umgebungsvariablen

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Authenticated data'),
  of('Public data')
);

data$.subscribe(console.log);
// Ausgabe: 'Authenticated data' oder 'Public data' (abhängig von der Bedingung bei Subscription)
```

### defer - Verzögerte Generierung bei Subscription

`defer` generiert ein Observable jedes Mal, wenn eine Subscription erfolgt. Dadurch kann das Verhalten des Observables basierend auf dem Zustand zum Zeitpunkt der Subscription geändert werden.

**Syntax**:
```typescript
defer(() => {
  // Wird bei Subscription ausgeführt
  return someObservable;
})
```

**Anwendungsfälle**:
- Observable-Generierung, die den neuesten Zustand bei Subscription widerspiegelt
- Generierung unterschiedlicher Zufallswerte bei jeder Subscription
- Ausführung unterschiedlicher Verarbeitung bei jeder Subscription

```typescript
import { defer, of } from 'rxjs';

// Aktuelle Zeit bei Subscription abrufen
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('First:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Second:', time));
}, 2000);

// Ausgabe:
// First: 2024-10-21T01:00:00.000Z
// Second: 2024-10-21T01:00:01.000Z
// ※ Unterschiedliche Zeiten werden ausgegeben, da die Subscription-Zeitpunkte unterschiedlich sind
```

## Unterschied zwischen iif und defer

| Merkmal | iif | defer |
|---------|-----|-------|
| **Optionen** | Auswahl zwischen zwei Observables | Generierung beliebiger Observables |
| **Evaluierungszeitpunkt** | Bedingung wird bei Subscription evaluiert | Funktion wird bei Subscription ausgeführt |
| **Verwendungszweck** | Bedingte Verzweigung | Dynamische Generierung |

## Verwendung in Pipelines

Bedingte Creation Functions können mit anderen Operatoren kombiniert werden.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Benutzerinformationen von Benutzer-ID abrufen
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Aktuellen Cache bei Subscription prüfen
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Konvertierung von Cold zu Hot

Wie in der obigen Tabelle gezeigt, **generieren alle bedingten Creation Functions Cold Observables**. Bei jeder Subscription werden Bedingungsevaluierung oder Generierungsfunktion ausgeführt.

Durch Verwendung von Multicasting-Operatoren (`share()`, `shareReplay()` usw.) können Cold Observables in Hot Observables konvertiert werden.

### Praxisbeispiel: Teilen von Ergebnissen bedingter Verzweigungen

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Bedingung wird bei jeder Subscription neu evaluiert
const coldIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Subscriber 1:', val));
coldIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Jeder Subscriber evaluiert die Bedingung unabhängig (unterschiedliche Ergebnisse möglich)

// 🔥 Hot - Ergebnis der Bedingungsevaluierung wird zwischen Subscribern geteilt
const hotIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Subscriber 1:', val));
hotIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Bedingung wird nur einmal evaluiert, Ergebnis wird geteilt
```

> [!TIP]
> Weitere Details finden Sie unter [Grundlegende Erstellung - Umwandlung von Cold zu Hot](/de/guide/creation-functions/basic/#umwandlung-von-cold-zu-hot).

## Nächste Schritte

Um die detaillierte Funktionsweise und praktische Beispiele jeder Creation Function zu lernen, klicken Sie auf die Links in der obigen Tabelle.

Durch das ergänzende Studium von [kombinierten Creation Functions](/de/guide/creation-functions/combination/) und [auswählenden/teilenden Creation Functions](/de/guide/creation-functions/selection/) können Sie ein Gesamtverständnis der Creation Functions erlangen.
