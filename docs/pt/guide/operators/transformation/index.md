---
description: Explica como processar e transformar dados em um stream usando operadores de transformação RxJS, desde transformações simples como map, scan, mergeMap, switchMap e concatMap até transformações assíncronas, buffering e janelamento. Padrões práticos que aproveitam a segurança de tipo do TypeScript serão introduzidos com abundantes exemplos de código.
---

# Operadores de Transformação

Os operadores de transformação são usados para transformar e processar dados dentro do pipeline RxJS.
Ao transformar valores em novas formas, eles permitem um controle mais flexível e poderoso sobre o fluxo de dados reativos.


## Lista de Operadores
### ◾ Transformações Simples de Valores

|Operador|Descrição|
|---|---|
|[map](./map)|Aplica uma função de transformação a cada valor|

### ◾ Acumulação

|Operador|Descrição|
|---|---|
|[scan](./scan)|Gera valores cumulativamente|
|[reduce](./reduce)|Gera apenas o resultado acumulado final|

### ◾ Emparelhamento e Agrupamento

|Operador|Descrição|
|---|---|
|[pairwise](./pairwise)|Processa dois valores consecutivos em pares|
|[groupBy](./groupBy)|Agrupa valores com base em uma chave|

### ◾ Transformação Assíncrona

|Operador|Descrição|
|---|---|
|[mergeMap](./mergeMap) |Transforma cada valor em um Observable e mescla em paralelo|
|[switchMap](./switchMap) |Muda para o Observable mais recente|
|[concatMap](./concatMap) |Executa cada Observable sequencialmente|
|[exhaustMap](./exhaustMap) |Ignora novas entradas enquanto executa|
|[expand](./expand) |Expande resultados recursivamente|

### ◾ Processamento em Lote

|Operador|Descrição|
|---|---|
|[buffer](./buffer) |Agrupa valores no momento de outro Observable|
|[bufferTime](./bufferTime) |Agrupa valores em intervalos regulares|
|[bufferCount](./bufferCount) |Agrupa valores por contagem especificada|
|[bufferWhen](./bufferWhen) |Buffering com condições de término controladas dinamicamente|
|[bufferToggle](./bufferToggle) |Buffering com controle independente de início e fim|
|[windowTime](./windowTime) |Divide em sub-Observables em intervalos regulares|


## Padrões Práticos de Transformação

Em aplicações do mundo real, o seguinte processamento é possível combinando operadores de transformação:

- Validação de entrada e feedback
- Controle ideal de requisições de API assíncronas
- Modelagem, agregação e normalização de dados
- Processamento em lote e agrupamento de fluxos de eventos

👉 Para mais informações: [Padrões Práticos de Transformação](./practical-use-cases)

## 🚨 Notas

Para evitar erros comuns ao usar operadores de transformação, consulte também:

- **[Efeitos colaterais em map](/pt/guide/anti-patterns/common-mistakes#5-side-effects-in-map)** - Use `map` como uma função pura
- **[Seleção inadequada de operador](/pt/guide/anti-patterns/common-mistakes#12-inappropriate-operator-selection)** - Uso adequado de operadores de ordem superior
