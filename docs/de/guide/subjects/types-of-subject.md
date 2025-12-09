---
description: "Die vier Arten von Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) und ihre jeweiligen Eigenschaften und Anwendungsszenarien werden erklärt. Unterscheidung nach Vorhandensein von Anfangswerten, Anzahl der Replay-Werte, Wertabruf nach Abschluss usw. wird anhand von TypeScript-Codebeispielen gelernt."
---

# Arten von Subject

Neben dem grundlegenden `Subject` bietet RxJS mehrere abgeleitete Klassen, die auf spezifische Anwendungsfälle spezialisiert sind. Jede hat unterschiedliche Verhaltensmerkmale, und durch ihre Verwendung in geeigneten Situationen wird effektiveres reaktives Programmieren möglich.

Hier werden die vier Hauptarten von Subject, ihre Eigenschaften und Anwendungsszenarien detailliert erklärt.

## Die vier grundlegenden Subject-Arten

| Art | Merkmale | Hauptanwendungsfälle |
|------|------|----------------|
| [`Subject`](#subject) | Das einfachste Subject<br>Empfängt nur Werte nach dem Abonnement | Event-Benachrichtigung, Multicast |
| [`BehaviorSubject`](#behaviorsubject) | Speichert den aktuellsten Wert und stellt ihn bei neuem Abonnement sofort bereit | State Management, aktuelle Werte von UI-Komponenten |
| [`ReplaySubject`](#replaysubject) | Gibt eine bestimmte Anzahl vergangener Werte an neue Abonnenten wieder | Operationshistorie, aktuelle Update-Informationen |
| [`AsyncSubject`](#asyncsubject) | Gibt nur den letzten Wert beim Abschluss aus | HTTP/API-Anfrageergebnisse |

## Standard-`Subject` {#subject}

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Der einfachste Typ von Subject, der nur nach dem Abonnement auftretende Werte empfängt.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Kein Anfangswert, empfängt beim Abonnement nichts
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Zweites Abonnement (empfängt nur Werte nach dem Abonnement)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Ausführungsergebnis
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Benötigt einen Anfangswert und speichert immer den aktuellsten Wert.
Neue Abonnenten erhalten beim Abonnement sofort den aktuellsten Wert.

```ts
import { BehaviorSubject } from 'rxjs';

// Mit Anfangswert 0 erstellen
const behaviorSubject = new BehaviorSubject<number>(0);

// Empfängt sofort den Anfangswert
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Zweites Abonnement (empfängt sofort aktuellsten Wert 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Ausführungsergebnis
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### Anwendungsbeispiele von BehaviorSubject

#### Verwaltung des Benutzerauthentifizierungsstatus

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// Anfangswert ist null (nicht angemeldet)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Anmeldestatus in Komponenten usw. überwachen
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Angemeldet: ${user.name}`);
  } else {
    console.log('Nicht angemeldet');
  }
});

// Anmeldeverarbeitung
function login(user: User) {
  currentUser$.next(user);
}

// Abmeldeverarbeitung
function logout() {
  currentUser$.next(null);
}

// Verwendungsbeispiel
console.log('Anwendungsstart');
// → Nicht angemeldet

login({ id: 'user123', name: 'Taro Yamada' });
// → Angemeldet: Taro Yamada

logout();
// → Nicht angemeldet
```

#### Ausführungsergebnis
```sh
Nicht angemeldet
Anwendungsstart
Angemeldet: Taro Yamada
Nicht angemeldet
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS Official: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Speichert eine bestimmte Anzahl vergangener Werte und sendet sie an neue Abonnenten erneut.
Puffergröße und Zeitfenster können eingestellt werden.

```ts
import { ReplaySubject } from 'rxjs';

// Puffert die letzten 3 Werte
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Abonnement starten (empfängt die letzten 3 Werte 2,3,4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Zweites Abonnement (empfängt die letzten 3 Werte 3,4,5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Ausführungsergebnis
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject mit Zeitfenster

Zeitbasiertes Puffern ist ebenfalls möglich.

```ts
import { ReplaySubject } from 'rxjs';

// Maximal 5 Werte und Werte innerhalb von 500ms puffern
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Nach 1000ms abonnieren (1 wird nicht empfangen, da Zeitfenster von 500ms überschritten)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Empfangen:', value));
  }, 1000);
}, 100);
```

#### Ausführungsergebnis
```
Empfangen: 2
```

### Anwendungsbeispiele von ReplaySubject

#### Verwaltung der Suchhistorie

```ts
import { ReplaySubject } from 'rxjs';

// Speichert die letzten 5 Suchanfragen
const searchHistory$ = new ReplaySubject<string>(5);

// Suchausführungsfunktion
function search(query: string) {
  console.log(`Suche ausführen: ${query}`);
  searchHistory$.next(query);
  // Tatsächliche Suchverarbeitung...
}

// Komponente zur Anzeige der Suchhistorie
function showSearchHistory() {
  console.log('--- Suchhistorie ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Verwendungsbeispiel
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Zeigt die letzten 5 (in diesem Fall 4) Suchhistorie-Einträge an
```

#### Ausführungsergebnis
```sh
Suche ausführen: TypeScript
Suche ausführen: RxJS
Suche ausführen: Angular
Suche ausführen: React
--- Suchhistorie ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS Official: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Subject, das nur den letzten Wert beim Abschluss ausgibt. Werte vor dem Abschluss werden nicht ausgegeben.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Unabhängig vom Zeitpunkt des Abonnements wird nur der letzte Wert empfangen
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // Beim Abschluss wird der letzte Wert (4) ausgegeben
```

#### Ausführungsergebnis
```
Observer 1: 4
Observer 2: 4
```

### Anwendungsbeispiele von AsyncSubject

#### Teilen von API-Anfrageergebnissen

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // API-Anfrage simulieren
  console.log(`API-Anfrage: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'Beispieldaten' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Verwendungsbeispiel
const data$ = fetchData('/api/users/1');

// Mehrere Komponenten können dasselbe Anfrageergebnis teilen
data$.subscribe(response => {
  console.log('Komponente 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Komponente 2:', response.data);
  });
}, 1500); // Kann auch nach Abschluss den Wert empfangen
```

#### Ausführungsergebnis
```sh
API-Anfrage: /api/users/1
Komponente 1: {id: 1, name: 'Beispieldaten'}
Komponente 2: {id: 1, name: 'Beispieldaten'}
```

## Vergleich und Auswahlhilfe für verschiedene Subjects

Nützliche Punkte bei der Auswahl des Subject-Typs werden zusammengefasst.

### Auswahlkriterien für Subject

|Typ|Auswahlkriterien|
|---|---|
|`Subject`|Für einfache Event-Benachrichtigungen oder Multicast-Verteilung|
|`BehaviorSubject`|<li>Wenn Anfangswert immer erforderlich </li><li>Daten, die den aktuellen Zustand repräsentieren (Benutzerstatus, Einstellungen, Flags usw.) </li><li>Aktuelle Werte von UI-Komponenten</li>|
|`ReplaySubject`|<li>Wenn kürzliche Operationshistorie gespeichert werden muss </li><li>Wenn später beitretenden Abonnenten vergangene Daten bereitgestellt werden sollen  </li><li>Gepufferte Datenstreams</li>|
|`AsyncSubject`|<li>Wenn nur das Endergebnis wichtig ist (wie API-Antworten) </li><li>Wenn Zwischenergebnisse unnötig sind und nur der Wert beim Abschluss geteilt werden soll</li>|

### Entscheidungsfluss zur Auswahl

1. Nur der letzte Wert beim Abschluss erforderlich ⇨ `AsyncSubject`
2. Letzte N Werte erforderlich ⇨ `ReplaySubject`
3. Aktueller Status/Wert immer erforderlich ⇨ `BehaviorSubject`
4. Alles andere (reine Event-Benachrichtigung usw.) ⇨ `Subject`

## Anwendungspatterns im Anwendungsdesign

### Beispiel für modulübergreifende Kommunikation

```ts
// Zustandsverwaltungsservice der gesamten Anwendung
class AppStateService {
  // Aktueller Benutzer (BehaviorSubject, da Anfangswert erforderlich)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Als read-only Observable veröffentlichen
  readonly user$ = this.userSubject.asObservable();

  // Benachrichtigung (Subject, da einfache Event-Benachrichtigung)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Kürzliche Suchen (ReplaySubject, da Historie erforderlich)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // API-Aufruf-Ergebnis-Cache (AsyncSubject, da nur Endergebnis erforderlich)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Methodenbeispiele
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // API-Ergebnis-Caching
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Tatsächlicher API-Aufruf
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }

    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## Zusammenfassung

RxJS Subject ist ein leistungsstarkes Tool, das verschiedene Anwendungsfälle abdeckt. Durch Verständnis der Eigenschaften jedes Typs und deren angemessene Nutzung können effiziente und wartbare reaktive Anwendungen erstellt werden.

- `Subject`: Am einfachsten, bietet grundlegende Multicast-Funktionalität
- `BehaviorSubject`: Speichert immer den aktuellen Zustand und stellt ihn sofort neuen Abonnenten bereit
- `ReplaySubject`: Speichert Historie der letzten Werte und stellt sie auch später beitretenden Abonnenten bereit
- `AsyncSubject`: Gibt nur den Endwert beim Abschluss aus

Die Auswahl des geeigneten `Subject` für State Management, Event-Benachrichtigung, Datenaustausch usw. ist der Schlüssel zu effektivem reaktiven Programmieren.
