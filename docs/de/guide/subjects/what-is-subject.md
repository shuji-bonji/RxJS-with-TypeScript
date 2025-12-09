---
description: Subject ist eine spezielle RxJS-Klasse, die sowohl Observable- als auch Observer-Eigenschaften besitzt. Es ermöglicht gleichzeitiges Veröffentlichen und Abonnieren von Daten und kann über Multicasting denselben Wert an mehrere Abonnenten verteilen. Mit TypeScript-Typparametern lassen sich praktische Patterns wie Event Bus oder State Management unter Beibehaltung der Typsicherheit implementieren.
---

# Was ist ein Subject

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Subject ist eine spezielle Art von Observable in RxJS. Während ein normales Observable einen unidirektionalen Datenfluss bereitstellt, ist ein Subject eine hybride Existenz, die sowohl die Eigenschaften von "Observable" als auch von "Observer" besitzt.

Subject hat folgende Eigenschaften:

- Kann Daten veröffentlichen (Observable-Funktion)
- Kann Daten abonnieren (Observer-Funktion)
- Kann denselben Wert an mehrere Abonnenten übermitteln (Multicast)
- Empfängt nur Werte, die nach dem Abonnement auftreten (Hot Observable-ähnliche Eigenschaft)


## Grundlegende Verwendung von Subject

```ts
import { Subject } from 'rxjs';

// Subject erstellen
const subject = new Subject<number>();

// Als Observer abonnieren
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Als Observable Werte veröffentlichen
subject.next(1); // Veröffentlicht Wert an beide Abonnenten
subject.next(2); // Veröffentlicht Wert an beide Abonnenten

// Neuen Abonnenten hinzufügen (verzögertes Abonnement)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Veröffentlicht Wert an alle Abonnenten

// Abschluss benachrichtigen
subject.complete();
```

#### Ausführungsergebnis
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Unterschied zu normalem Observable

Subject ist ein **Hot Observable** und unterscheidet sich in folgenden Punkten von normalen Cold Observables:

- Daten werden unabhängig vom Vorhandensein von Abonnements veröffentlicht
- Kann denselben Wert an mehrere Abonnenten teilen (Multicast)
- Kann Werte von außen mit `.next()` veröffentlichen
- Vergangene Werte werden nicht gespeichert, nur Werte nach dem Abonnement werden empfangen


## Subject und Multicasting

Eine wichtige Funktion von Subject ist "Multicasting".
Dies ist die Fähigkeit, eine Datenquelle effizient an mehrere Abonnenten zu verteilen.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Datenquelle
const source$ = interval(1000).pipe(take(3));

// Subject für Multicast
const subject = new Subject<number>();

// Quelle mit Subject verbinden
source$.subscribe(subject); // Subject fungiert als Abonnent

// Mehrere Beobachter abonnieren das Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Ausführungsergebnis
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

Dieses Pattern wird auch Single-Source-Multicast genannt und wird verwendet, um eine Datenquelle effizient an mehrere Abonnenten zu verteilen.


## Zwei Verwendungsweisen von Subject

Subject hat hauptsächlich zwei Verwendungsweisen. Jede hat unterschiedliche Zwecke und Verhaltensweisen.

### 1. Pattern zum selbstständigen Aufrufen von `.next()`

Subject wird als **Hauptakteur der Datenveröffentlichung (Observable)** verwendet.
Dieses Pattern eignet sich für "explizite Werteübermittlung" wie Event-Benachrichtigungen oder Statusaktualisierungen.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Ausgabe:
// Observer A: Hello
// Observer A: World
```

---

### 2. Pattern zur Weiterleitung von Observables (Multicast)

Subject fungiert als **Observer, der Werte von einem Observable empfängt und weiterleitet**.
Diese Verwendung ist praktisch, um **Cold Observables in Hot umzuwandeln und zu multicasten**.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (Weiterleitung)
source$.subscribe(subject);

// Subject → Verteilung an mehrere Abonnenten
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Ausgabe:
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> Wenn Sie `.next()` selbst aufrufen, ist es wie "eine Person, die selbst spricht"; wenn Sie von einem Observable empfangen und weiterleiten, ist es wie "eine Person, die die Worte anderer über ein Mikrofon verstärkt" - diese Vorstellung erleichtert das Verständnis.


## Praktische Anwendungsfälle von Subject

Subject ist besonders nützlich in folgenden Szenarien:

1. **State Management** - Teilen und Aktualisieren des Anwendungszustands
2. **Event Bus** - Kommunikation zwischen Komponenten
3. **HTTP-Antworten teilen** - Ergebnisse desselben API-Aufrufs in mehreren Komponenten teilen
4. **Zentrales UI-Event-Management** - Verschiedene UI-Operationen an einem Ort verarbeiten

#### Beispiel: Event-Bus-Implementierung
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Anwendungsweiter Event Bus
const eventBus = new Subject<AppEvent>();

// Bestimmten Event-Typ abonnieren
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('Benutzer-Login:', event.payload);
});

// Anderen Event-Typ abonnieren
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Datenaktualisierung:', event.payload);
});

// Events veröffentlichen
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Ausführungsergebnis
```
Benutzer-Login: {userId: '123', username: 'test_user'}
Datenaktualisierung: {items: Array(3)}
```

## Zusammenfassung

Subject ist ein wichtiger Bestandteil im RxJS-Ökosystem und erfüllt folgende Rollen:

- Besitzt sowohl Observer- als auch Observable-Eigenschaften
- Bietet Mittel zur Umwandlung von Cold Observables in Hot
- Verteilt denselben Datenstrom effizient an mehrere Abonnenten
- Erleichtert die Kommunikation zwischen Komponenten und Services
- Bietet Grundlage für State Management und Event-Verarbeitung

## 🔗 Verwandte Abschnitte

- **[Häufige Fehler und Lösungen](/de/guide/anti-patterns/common-mistakes#1-subject-の外部公開)** - Best Practices zur Vermeidung von Subject-Missbrauch
- **[Arten von Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject usw.
