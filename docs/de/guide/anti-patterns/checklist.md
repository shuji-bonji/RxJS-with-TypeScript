---
description: "Checkliste zur Vermeidung von Antipatterns beim Schreiben von RxJS-Code. 16 Best Practices für korrekte Subscription-Kündigung, Subject-Nutzung, Fehlerbehandlung und Memory-Leak-Prävention."
---

# Checkliste zur Vermeidung von Anti-Patterns

Verwenden Sie diese Checkliste, um Ihren RxJS-Code auf Anti-Patterns zu überprüfen. Klicken Sie auf jeden Punkt, um eine detaillierte Erklärung und Beispielcode zu erhalten.

## Checkliste

### 🔴 Kritische Probleme vermeiden

| Check | Punkt | Hinweis |
|:---:|---|---|
| <input type="checkbox" /> | **[Subject über asObservable() veröffentlichen](./common-mistakes#1-subject-外部公開)** | `Subject` nicht direkt exportieren, sondern als Observable über `asObservable()` bereitstellen<br>Zustandsänderungen nur über dedizierte Methoden ermöglichen |
| <input type="checkbox" /> | **[Verschachtelte subscribe vermeiden](./common-mistakes#2-ネストした-subscribe-コールバック地獄)** | Kein weiteres `subscribe` innerhalb eines `subscribe` aufrufen<br>Mit `switchMap`, `mergeMap`, `concatMap` etc. flach machen |
| <input type="checkbox" /> | **[Unendliche Streams immer abbestellen](./common-mistakes#3-unsubscribe-忘れ-メモリリーク)** | Unendliche Streams wie Event-Listener immer abbestellen<br>`takeUntil`-Muster oder `Subscription`-Verwaltung |
| <input type="checkbox" /> | **[shareReplay-Einstellungen explizit angeben](./common-mistakes#4-sharereplay-の誤用)** | Format `shareReplay({ bufferSize: 1, refCount: true })` verwenden<br>Referenzzählung aktivieren um Memory-Leaks zu verhindern |
| <input type="checkbox" /> | **[Verschachtelte if-Anweisungen in subscribe vermeiden](./subscribe-if-hell)** | Komplexe Verzweigungen (3+ Verschachtelungen) in `subscribe` vermeiden<br>Deklarativ mit `filter`, `iif`, `partition` etc. schreiben |

### 🟡 Probleme die Aufmerksamkeit erfordern

| Check | Punkt | Hinweis |
|:---:|---|---|
| <input type="checkbox" /> | **[map ist reine Funktion, Seiteneffekte in tap](./common-mistakes#5-map-での副作用)** | In `map` keine Zustandsänderungen oder Log-Ausgaben<br>Seiteneffekte explizit mit `tap`-Operator trennen |
| <input type="checkbox" /> | **[Cold/Hot richtig unterscheiden](./common-mistakes#6-cold-hot-observable-の違いの無視)** | HTTP-Requests etc. mit `shareReplay` in Hot umwandeln<br>Entscheiden ob pro Subscription oder geteilt ausführen |
| <input type="checkbox" /> | **[Promise mit from konvertieren](./common-mistakes#7-promise-と-observable-の不適切な混在)** | Promise und Observable nicht mischen<br>Mit `from()` in Observable konvertieren und einheitlich verarbeiten |
| <input type="checkbox" /> | **[Hochfrequente Events kontrollieren](./common-mistakes#8-バックプレッシャーの無視)** | Sucheingabe mit `debounceTime`, Scrollen mit `throttleTime` kontrollieren<br>Duplikate mit `distinctUntilChanged` ausschließen |

### 🔵 Verbesserung der Codequalität

| Check | Punkt | Hinweis |
|:---:|---|---|
| <input type="checkbox" /> | **[Fehler richtig behandeln](./common-mistakes#9-エラーの握りつぶし)** | Fehler mit `catchError` abfangen und richtig verarbeiten<br>Benutzerfreundliche Fehlermeldungen anzeigen<br>Bei Bedarf mit `retry` / `retryWhen` wiederholen |
| <input type="checkbox" /> | **[DOM-Events richtig freigeben](./common-mistakes#10-dom-イベントサブスクリプションのリーク)** | `fromEvent`-Subscriptions immer beenden<br>Bei Komponenten-Zerstörung automatisch mit `takeUntil` abbestellen |
| <input type="checkbox" /> | **[Typsicherheit gewährleisten](./common-mistakes#11-型安全性の欠如-any-の多用)** | Interfaces und Type-Aliase definieren<br>Typparameter für `Observable<T>` explizit angeben<br>TypeScript-Typinferenz nutzen |
| <input type="checkbox" /> | **[Passende Operatoren wählen](./common-mistakes#12-不適切なオペレーター選択)** | Suche: `switchMap`, parallel: `mergeMap`<br>Sequentiell: `concatMap`, Doppelklick-Schutz: `exhaustMap` |
| <input type="checkbox" /> | **[Einfache Verarbeitung braucht kein RxJS](./common-mistakes#13-過度な複雑化)** | Array-Verarbeitung etc. reicht normales JavaScript<br>RxJS für asynchrone Verarbeitung und Event-Streams verwenden |
| <input type="checkbox" /> | **[Zustand reaktiv verwalten](./common-mistakes#14-subscribe-内での状態変更)** | Zustand mit `BehaviorSubject` oder `scan` verwalten<br>`subscribe` als finalen Trigger verwenden |
| <input type="checkbox" /> | **[Tests schreiben](./common-mistakes#15-テストの欠如)** | Marble-Tests mit `TestScheduler` durchführen<br>Asynchrone Verarbeitung synchron testbar machen |

## Verwendung

### 1. Bei Code-Reviews

Nach dem Schreiben von neuem Code diese Checkliste für Selbst-Reviews nutzen.

### 2. Bei Pull Requests

Diese Checkliste in Pull-Request-Vorlagen einbinden für gemeinsame Überprüfungskriterien.

### 3. Regelmäßige Überprüfung

Diese Checkliste regelmäßig auf bestehende Codebasen anwenden um Anti-Pattern-Einschleichung zu erkennen.

### 4. Team-Sharing

Mit Teammitgliedern teilen um RxJS Best Practices zu vereinheitlichen.

## Verwandte Ressourcen

- **[Häufige Fehler und Lösungen](./common-mistakes)** - Detaillierte Erklärungen und Codebeispiele für jedes Anti-Pattern
- **[Anti-Pattern-Sammlung Übersicht](./index)** - Anti-Pattern-Liste und Lernpfad
- **[Fehlerbehandlung](/de/guide/error-handling/strategies)** - Best Practices für Fehlerbehandlung
- **[Testmethoden](/de/guide/testing/unit-tests)** - Wie man RxJS-Code testet

## Tipps zur Checklisten-Nutzung

1. **Nicht alle Punkte auf einmal perfekt machen wollen**
   - Zuerst kritische Probleme (🔴) priorisieren
   - Schrittweise verbessern

2. **Prioritäten im Team festlegen**
   - Wichtigkeit je nach Projekteigenschaften anpassen
   - Angepasste Checklisten erstellen

3. **Automatisierung erwägen**
   - Automatische Checks mit statischen Analysetools wie ESLint
   - In CI/CD-Pipeline integrieren

4. **Regelmäßig aktualisieren**
   - Bei RxJS-Versions-Upgrades aktualisieren
   - Team-Erfahrungswissen einarbeiten

---

**Wichtig**: Diese Checkliste dient nicht dem Schreiben von perfektem Code, sondern der Vermeidung häufiger Probleme. Bitte je nach Projektkontext flexibel einsetzen.
