---
name: rxjs-glossary
description: |
  RxJS-with-TypeScript プロジェクトの翻訳用語集。固有名詞（翻訳しない用語）と、各言語 (fr/de/it/es/nl/pt) の
  定型訳テーブル（テーブルヘッダー、よく出るフレーズ、コード内日本語フラグメント）。
  DeepL 翻訳前後で用語を保護・統一するための辞書として、rxjs-vitepress-i18n ワークフローと併用する。
---

# RxJS-with-TypeScript 翻訳用語集

## 翻訳しない固有名詞（全言語共通）

DeepL は固有名詞を意訳してしまうので、翻訳前にプレースホルダーで保護するか、翻訳後に修正する。

### RxJS 関数・型

```
forkJoin, combineLatest, withLatestFrom, mergeWith, concatWith, raceWith,
combineLatestWith, merge, concat, zip, race, partition,
Observable, Subject, BehaviorSubject, ReplaySubject, AsyncSubject,
Observer, Subscription, Subscriber, Scheduler,
of, from, fromEvent, fromFetch, ajax, interval, timer, range, generate,
iif, defer, scheduled, using,
map, filter, debounceTime, throttleTime, distinctUntilChanged, distinct,
mergeMap, switchMap, concatMap, exhaustMap, take, takeUntil, takeWhile,
skip, skipUntil, skipWhile, first, last, single, find, findIndex,
elementAt, ignoreElements, sample, sampleTime, audit, auditTime,
buffer, bufferTime, bufferCount, bufferToggle, bufferWhen,
window, windowTime, windowCount, windowToggle, windowWhen,
tap, delay, timeout, retry, retryWhen, catchError, finalize, throwIfEmpty,
share, shareReplay, publish, multicast, refCount,
toArray, reduce, scan, mergeScan, pairwise, groupBy,
materialize, dematerialize, every, isEmpty, count
```

### TypeScript / JavaScript

```
TypeScript, JavaScript, Promise, fetch, async, await,
WebSocket, EventTarget, Event, MouseEvent, KeyboardEvent,
HTMLElement, HTMLInputElement, HTMLButtonElement
```

### 概念

```
RxJS, Reactive, Reactive Programming, Hot, Cold, Multicast,
pipe, subscribe, unsubscribe, complete, next, error,
Creation Function, Pipeable Operator,
Angular, React, Vue, Signals, NgRx
```

### 文中保護のヒント

`backtick` で囲まれた用語は DeepL でも比較的保護されやすい。翻訳前に重要な固有名詞を `\`forkJoin\`` 形式に置換しておくと精度が上がる。

## テーブルヘッダー定型訳

| JA | fr | de | it | es | nl | pt |
|----|----|----|----|----|----|-----|
| 特徴 | Caractéristique | Merkmal | Caratteristica | Característica | Kenmerk | Característica |
| 説明 | Description | Beschreibung | Descrizione | Descripción | Beschrijving | Descrição |
| ユースケース | Cas d'usage | Anwendungsfall | Caso d'uso | Caso de uso | Gebruikscase | Caso de uso |
| 例 | Exemple | Beispiel | Esempio | Ejemplo | Voorbeeld | Exemplo |
| オペレーター | Opérateur | Operator | Operatore | Operador | Operator | Operador |
| 入力 | Entrée | Eingabe | Input | Entrada | Invoer | Entrada |
| 出力 | Sortie | Ausgabe | Output | Salida | Uitvoer | Saída |
| 結果 | Résultat | Ergebnis | Risultato | Resultado | Resultaat | Resultado |
| 注意 | Attention | Hinweis | Attenzione | Atención | Let op | Atenção |
| エラー | Erreur | Fehler | Errore | Error | Fout | Erro |
| 期間 | Période | Zeitraum | Periodo | Periodo | Periode | Período |
| 完了条件 | Condition de complétion | Abschlussbedingung | Condizione di completamento | Condición de finalización | Voltooiingsvoorwaarde | Condição de conclusão |
| 出力タイミング | Timing d'émission | Ausgabe-Timing | Timing di emissione | Timing de emisión | Emissie-timing | Timing de emissão |
| 出力値 | Valeur émise | Emittierter Wert | Valore emesso | Valor emitido | Geëmitteerde waarde | Valor emitido |
| 選択基準 | Critère de sélection | Auswahlkriterium | Criterio di selezione | Criterio de selección | Selectiecriterium | Critério de seleção |
| 主な用途 | Cas d'usage principal | Hauptanwendung | Uso principale | Uso principal | Hoofdgebruik | Uso principal |
| 無限ストリーム | Flux infini | Unendlicher Stream | Stream infinito | Stream infinito | Oneindige stream | Stream infinito |

## よく出る短文・フレーズ

| JA | fr | de | it | es | nl | pt |
|----|----|----|----|----|----|-----|
| 動的な期間制御 | Contrôle dynamique de la période | Dynamische Periodensteuerung | Controllo dinamico del periodo | Control dinámico del periodo | Dynamische periode-controle | Controle dinâmico de período |
| カスタム Observable | Observable personnalisé | Benutzerdefinierte Observable | Observable personalizzato | Observable personalizado | Aangepaste Observable | Observable personalizado |
| 固定時間 | Temps fixe | Feste Zeit | Tempo fisso | Tiempo fijo | Vaste tijd | Tempo fixo |
| ミリ秒 | millisecondes | Millisekunden | millisecondi | milisegundos | milliseconden | milissegundos |
| 期間内の最後の値 | la dernière valeur de la période | letzter Wert der Periode | l'ultimo valore del periodo | el último valor del periodo | de laatste waarde van de periode | o último valor do período |
| 期間内の最新の値 | la valeur la plus récente de la période | neuester Wert der Periode | il valore più recente del periodo | el valor más reciente del periodo | de meest recente waarde van de periode | o valor mais recente do período |
| すべて完了後に1回だけ | Une seule fois après complétion de tous | Nur einmal nach Abschluss aller | Solo una volta dopo il completamento di tutti | Solo una vez tras completarse todos | Slechts één keer na voltooiing van alle | Apenas uma vez após conclusão de todos |
| 値が更新されるたびに | À chaque mise à jour d'une valeur | Bei jeder Wertaktualisierung | A ogni aggiornamento di un valore | Cada vez que se actualiza un valor | Bij elke waarde-update | A cada atualização de valor |
| すべてのObservableが完了 | Tous les Observables doivent se compléter | Alle Observables sind abgeschlossen | Tutti gli Observable completati | Todos los Observables completados | Alle Observables voltooid | Todos os Observables concluídos |
| API並列取得 | Acquisition parallèle d'API | Paralleler API-Abruf | Acquisizione parallela di API | Adquisición paralela de APIs | Parallelle API-acquisitie | Aquisição paralela de APIs |
| 初期データロード | Chargement initial des données | Initiales Laden der Daten | Caricamento iniziale dei dati | Carga inicial de datos | Initieel laden van data | Carregamento inicial de dados |
| フォーム監視 | Surveillance de formulaire | Formularüberwachung | Monitoraggio form | Monitorización de formulario | Formuliermonitoring | Monitoramento de formulário |
| リアルタイム同期 | Synchronisation en temps réel | Echtzeit-Synchronisation | Sincronizzazione in tempo reale | Sincronización en tempo real | Realtime synchronisatie | Sincronização em tempo real |
| 使用可能 | Utilisable | Verwendbar | Utilizzabile | Utilizable | Bruikbaar | Utilizável |
| 使用不可 | Inutilisable | Nicht verwendbar | Non utilizzabile | No utilizable | Niet bruikbaar | Não utilizável |
| 完了しない | ne se complète pas | wird nicht abgeschlossen | non si completa | no se completa | voltooit niet | não se completa |
| 値を出力 | émet des valeurs | gibt Werte aus | emette valori | emite valores | emitteert waarden | emite valores |

## コード内日本語の言語別マッピング (`code_jp`)

これは `forkJoin-vs-combineLatest.md` と `audit.md` で実際に使用したマッピング。
他のファイルでも共通して出現するフラグメントなので、再利用可能。

### 共通フラグメント

| JA | fr | de | it | es | nl | pt |
|----|----|----|----|----|----|-----|
| 値 | valeur | Wert | valore | valor | waarde | valor |
| 発行 | émission | Ausgabe | emissione | emisión | uitgifte | emissão |
| 出力 | Sortie | Ausgabe | Output | Salida | Uitvoer | Saída |
| 例 | Ex. | Bsp. | Es. | Ej. | Bv. | Ex. |
| はい | oui | ja | sì | sí | ja | sim |
| いいえ | non | nein | no | no | nee | não |
| 比較 | Comparaison | Vergleich | Confronto | Comparación | Vergelijking | Comparação |
| 作成 | Créer | Erstellen | Creare | Crear | Maken | Criar |
| 待機 | en attente | warten | in attesa | esperando | wachten | aguardando |
| 期間 | période | Periode | periodo | periodo | periode | período |
| 結果 | résultat | Ergebnis | risultato | resultado | resultaat | resultado |
| 完了 | complété | abgeschlossen | completato | completado | voltooid | concluído |
| 並列取得 | acquisition parallèle | paralleler Abruf | acquisizione parallela | adquisición paralela | parallelle acquisitie | aquisição paralela |
| 初期データロード | chargement initial des données | initiales Laden | caricamento iniziale dati | carga inicial de datos | initieel data laden | carregamento inicial de dados |
| フォーム監視 | surveillance de formulaire | Formularüberwachung | monitoraggio form | monitorización de formulario | formuliermonitoring | monitoramento de formulário |
| リアルタイム同期 | synchronisation en temps réel | Echtzeit-Synchronisation | sincronizzazione in tempo reale | sincronización en tempo real | realtime synchronisatie | sincronização em tempo real |
| 動的な期間 | période dynamique | dynamischer Zeitraum | periodo dinamico | periodo dinámico | dynamische periode | período dinâmico |
| 動的サンプリング | échantillonnage dynamique | dynamisches Sampling | campionamento dinamico | muestreo dinámico | dynamische bemonstering | amostragem dinâmica |
| 高負荷 | charge élevée | hohe Last | carico alto | carga alta | hoge belasting | carga alta |
| 中負荷 | charge moyenne | mittlere Last | carico medio | carga media | gemiddelde belasting | carga média |
| 低負荷 | charge faible | niedrige Last | carico basso | carga baja | lage belasting | carga baixa |
| 良い例 | Bon exemple | Gutes Beispiel | Esempio positivo | Buen ejemplo | Goed voorbeeld | Bom exemplo |
| 悪い例 | Mauvais exemple | Schlechtes Beispiel | Esempio negativo | Mal ejemplo | Slecht voorbeeld | Exemplo ruim |
| 最大 | Max. | Max. | Max. | Máx. | Max. | Máx. |
| 最初 | Premier | Erste | Primo | Primero | Eerste | Primeiro |
| 最後 | Dernier | Letzte | Ultimo | Último | Laatste | Último |
| 定期的 | Périodique | Periodisch | Periodico | Periódico | Periodiek | Periódico |
| 出力エリア | zone d'affichage | Ausgabebereich | area di output | área de salida | uitvoergebied | área de saída |

## frontmatter / callout 定型訳

### Production warning callout (Phase 3 で全 6 言語に展開)

| 言語 | 翻訳 |
|------|------|
| fr | `> [!WARNING] Attention en code de production\n> L'exemple ci-dessus omet la désinscription de \`fromEvent\` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec \`takeUntil(destroy$)\`, \`take(N)\`, ou \`Subscription.unsubscribe()\`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)` |
| de | `> [!WARNING] Hinweis für Produktionscode\n> Das obige Beispiel lässt die Abmeldung von \`fromEvent\` zur Vereinfachung der Erklärung weg. ...` |
| it | `> [!WARNING] Attenzione in codice di produzione\n> L'esempio sopra omette la disiscrizione di \`fromEvent\` per semplificare la spiegazione. ...` |
| es | `> [!WARNING] Atención en código de producción\n> El ejemplo anterior omite la cancelación de suscripción de \`fromEvent\`. ...` |
| nl | `> [!WARNING] Let op in productiecode\n> Het bovenstaande voorbeeld laat het afmelden van \`fromEvent\` weg om de uitleg te vereenvoudigen. ...` |
| pt | `> [!WARNING] Atenção em código de produção\n> O exemplo acima omite o cancelamento da inscrição de \`fromEvent\` para simplificar a explicação. ...` |

### retryWhen 非推奨 callout

| 言語 | 翻訳 |
|------|------|
| fr | `> [!IMPORTANT] retryWhen est déprécié\n> L'ancien opérateur \`retryWhen\` (déprécié en v7.3, prévu pour suppression en v8) doit être remplacé par la forme \`retry({ count, delay })\`...` |
| de | `> [!IMPORTANT] retryWhen ist deprecated\n> Der alte \`retryWhen\`-Operator (in v7.3 deprecated, in v8 zur Entfernung vorgesehen) sollte durch die \`retry({ count, delay })\`-Form ersetzt werden...` |
| it | `> [!IMPORTANT] retryWhen è deprecato\n> Il vecchio operatore \`retryWhen\` (deprecato in v7.3, previsto per la rimozione in v8) deve essere sostituito con la forma \`retry({ count, delay })\`...` |
| es | `> [!IMPORTANT] retryWhen está obsoleto\n> El antiguo operador \`retryWhen\` (obsoleto en v7.3, previsto para eliminación en v8) debe reemplazarse con la forma \`retry({ count, delay })\`...` |
| nl | `> [!IMPORTANT] retryWhen is deprecated\n> De oude \`retryWhen\` operator (deprecated in v7.3, gepland voor verwijdering in v8) moet vervangen worden door de \`retry({ count, delay })\` vorm...` |
| pt | `> [!IMPORTANT] retryWhen está deprecated\n> O antigo operador \`retryWhen\` (deprecated em v7.3, previsto para remoção em v8) deve ser substituído pela forma \`retry({ count, delay })\`...` |

## glossary.json 形式

ローカルスクリプトから読み込む用の JSON 形式：

```json
{
  "no_translate": ["forkJoin", "combineLatest", "Observable", "Subject", "..."],
  "fr": {
    "table_headers": {"特徴": "Caractéristique", "ユースケース": "Cas d'usage", "..."},
    "code_jp": {"値": "valeur", "発行": "émission", "..."},
    "callouts": {"production_warning": "> [!WARNING] Attention en code de production\n..."}
  },
  "de": {...},
  "it": {...},
  "es": {...},
  "nl": {...},
  "pt": {...}
}
```

## 配置場所

- Skill 本体 (このファイル): `.claude/skills/rxjs-glossary/SKILL.md`
- JSON 形式の辞書: `.claude/skills/rxjs-glossary/glossary.json` (スクリプトから読み込み)

## 学習・更新方針

ローカルスクリプトで翻訳実行時、新しい日本語フラグメントに遭遇したら：
1. DeepL で翻訳
2. 翻訳結果を `glossary.json` の該当言語の `code_jp` に追記
3. 翌回からはローカル辞書を優先（DeepL 消費削減）

## 関連 Skill

- `rxjs-vitepress-i18n` — このグロサリーを使う翻訳ワークフロー本体
