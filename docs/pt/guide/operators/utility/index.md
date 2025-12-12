---
description: Utility operators são um grupo de operadores auxiliares no RxJS que são responsáveis por controlar efeitos colaterais, processamento de atraso, gerenciamento de inscrição, etc.
---

# Utility Operators

Utility operators no RxJS são um grupo de operadores responsáveis pelo **processamento auxiliar de streams (efeitos colaterais, controle de estado, suporte de UI, etc.)** em vez do propósito principal de conversão ou filtragem de dados.

Nesta página, os operadores são categorizados por finalidade conforme mostrado abaixo, e uma lista é fornecida para confirmar seu uso básico.
Para uso detalhado e exemplos práticos, consulte as respectivas páginas ou [Casos de Uso Práticos](./practical-use-cases.md).


## Lista de Operadores (por Finalidade)

### ◾ Efeitos Colaterais e Controle de Estado

| Operador | Descrição | Frequentemente Combinado Com |
|--------------|------|------------------|
| [tap](./tap.md) | Executa efeitos colaterais sem alterar valores (saída de log, atualizações de UI, etc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Executa processamento de limpeza quando o stream termina | `tap`, `catchError` |


### ◾ Controle de Timing e Atraso

| Operador | Descrição | Frequentemente Combinado Com |
|--------------|------|------------------|
| [delay](./delay.md) | Atrasa a emissão de cada valor por um tempo especificado | `tap`, `concatMap` |
| [timeout](./timeout.md) | Gera um erro se a emissão exceder um determinado tempo | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | Encerra a inscrição quando o Observable especificado notifica | `interval`, `fromEvent` |


### ◾ Valor Inicial, Repetição, Conversão de Array, etc.

| Operador | Descrição | Frequentemente Combinado Com |
|--------------|------|------------------|
| [startWith](./startWith.md) | Emite um valor inicial no início do stream | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Reinscreve-se no stream inteiro após a conclusão | `tap`, `delay` |
| [retry](./retry.md) | Tenta novamente em caso de erro | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Emite todos os valores no stream como um único array (na conclusão) | `concatMap`, `take` |


## Observações

- Diferença entre `retry` e `repeat`:
  - `retry`: **Tenta novamente em caso de erro**
  - `repeat`: **Tenta novamente após conclusão bem-sucedida**
- `toArray` não gera um valor a menos que seja concluído, então é comumente usado com `take()` e assim por diante.
