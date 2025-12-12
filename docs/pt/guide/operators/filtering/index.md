---
description: Os operadores de filtragem do RxJS são usados para extrair apenas os dados necessários do stream com base em condições e tempo, contribuindo para a melhoria do desempenho.
---

# Operadores de Filtragem

Os operadores de filtragem do RxJS são ferramentas importantes para selecionar apenas os dados necessários de um stream e não deixar passar dados desnecessários.
Isso melhora muito a eficiência e o desempenho da aplicação.

Os operadores de filtragem são um conjunto de operadores RxJS que classificam os valores em um stream, permitindo que apenas aqueles que atendem a determinados critérios passem.
Ao controlar o fluxo de dados e processar apenas os valores necessários, você pode construir um pipeline de processamento de dados eficiente.


## Lista de Operadores
### ◾ Operadores de Filtragem Básicos

| Operador | Descrição |
|:---|:---|
| [filter](./filter) | Deixa passar apenas valores que correspondem à condição |
| [take](./take) | Obtém apenas o número especificado de primeiros valores |
| [takeLast](./takeLast) | Obtém o número especificado de últimos valores |
| [takeWhile](./takeWhile) | Obtém valores enquanto a condição é atendida |
| [skip](./skip) | Pula o número especificado de primeiros valores |
| [skipLast](./skipLast) | Pula o número especificado de últimos valores |
| [skipWhile](./skipWhile) | Pula valores enquanto a condição é atendida |
| [skipUntil](./skipUntil) | Pula valores até que outro Observable emita |
| [first](./first) | Obtém o primeiro valor, ou o primeiro valor que atende a uma condição |
| [last](./last) | Obtém o último valor, ou o último valor que atende a uma condição |
| [elementAt](./elementAt) | Obtém o valor em um índice especificado |
| [find](./find) | Encontra o primeiro valor que atende a uma condição |
| [findIndex](./findIndex) | Obtém o índice do primeiro valor que atende a uma condição |
| [ignoreElements](./ignoreElements) | Ignora todos os valores e só passa completions/erros |


### ◾ Operadores de Filtragem Baseados em Tempo

| Operador | Descrição |
|:---|:---|
| [debounceTime](./debounceTime) | Emite o último valor se nenhuma entrada for recebida por um tempo especificado |
| [throttleTime](./throttleTime) | Passa o primeiro valor e ignora novos valores pelo tempo especificado |
| [auditTime](./auditTime) | Emite o último valor após um tempo especificado |
| [audit](./audit) | Controla o período com um Observable personalizado e emite o último valor |
| [sampleTime](./sampleTime) | Amostra o último valor em intervalos de tempo especificados |


### ◾ Operadores de Filtragem Baseados em Condição

| Operador | Descrição |
|:---|:---|
| [distinct](./distinct) | Remove todos os valores duplicados (emite apenas valores únicos) |
| [distinctUntilChanged](./distinctUntilChanged) | Remove valores duplicados consecutivos |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Detecta apenas mudanças em propriedades específicas |


## Casos de Uso Práticos

- [Casos de Uso Práticos](./practical-use-cases.md) apresenta exemplos práticos de combinação de múltiplos operadores de filtragem (pesquisa em tempo real, rolagem infinita, etc.).
