using Plang.Compiler.TypeChecker.AST;
using Plang.Compiler.TypeChecker.AST.Declarations;
using Plang.Compiler.TypeChecker.AST.Expressions;
using Plang.Compiler.TypeChecker.AST.Statements;
using Plang.Compiler.TypeChecker.Types;
using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Linq;

namespace Plang.Compiler.TypeChecker
{
    public class FunctionBodyVisitor : PParserBaseVisitor<object>
    {
        private readonly ITranslationErrorHandler handler;
        private readonly Machine machine;
        private readonly Function method;

        private FunctionBodyVisitor(ITranslationErrorHandler handler, Machine machine, Function method)
        {
            this.handler = handler;
            this.machine = machine;
            this.method = method;
        }

        public static void PopulateMethod(ITranslationErrorHandler handler, Function fun)
        {
            Contract.Requires(fun.Body == null);
            FunctionBodyVisitor visitor = new FunctionBodyVisitor(handler, fun.Owner, fun);
            visitor.Visit(fun.SourceLocation);
        }

        public override object VisitAnonEventHandler(PParser.AnonEventHandlerContext context)
        {
            return Visit(context.functionBody());
        }

        public override object VisitNoParamAnonEventHandler(PParser.NoParamAnonEventHandlerContext context)
        {
            return Visit(context.functionBody());
        }

        public override object VisitPFunDecl(PParser.PFunDeclContext context)
        {
            return Visit(context.functionBody());
        }

        public override object VisitForeignFunDecl(PParser.ForeignFunDeclContext context)
        {
            return null;
        }

        public override object VisitEventDrivenRTAModule(PParser.EventDrivenRTAModuleContext context)
        {
            List<IPStmt> statements = new List<IPStmt>();
            if (method.Role.HasFlag(FunctionRole.EntryHandler) && machine.isStartedVar != null)
            {
                statements.Add(new AssignStmt(context, new VariableAccessExpr(context, machine.isStartedVar), new BoolLiteralExpr(context, false)));
                statements.Add(new AssignStmt(context, new VariableAccessExpr(context, machine.decisionPeriodCountVar), new IntLiteralExpr(context, 0)));
                method.Body = new CompoundStmt(context, statements);
                return null;
            }
            List<IPExpr> emptyArgsList = new List<IPExpr>();
            List<Function> rtaControllers = method.RTAControllers;
            List<string> rtaControllerNames = method.RTAControllerNames;
            Function rtaDecisionModule = method.RTADecisionModule;
            Dictionary<string, int> rtaDecisionPeriods = method.RTADecisionPeriods;

            Variable isStarted = machine.isStartedVar;

            List<IPStmt> isStartedCheckThenStatements = new List<IPStmt>();
            IPStmt isStartedTrueAssignment = new AssignStmt(context, new VariableAccessExpr(context, isStarted), new BoolLiteralExpr(context, true));
            isStartedCheckThenStatements.Add(isStartedTrueAssignment);

            method.AddCallee(rtaDecisionModule);
            IPExpr decisionModuleCall = new FunCallExpr(context, rtaDecisionModule, emptyArgsList);

            Variable mode = machine.modeVar;
            AssignStmt modeAssignment = new AssignStmt(context, new VariableAccessExpr(context, mode), decisionModuleCall);
            isStartedCheckThenStatements.Add(modeAssignment);

            Variable decisionPeriod = machine.decisionPeriodVar;

            Variable decisionPeriodCount = machine.decisionPeriodCountVar;

            IPExpr controllerCheckLhs = new VariableAccessExpr(context, mode);
            IPExpr controllerCheckRhs = new StringExpr(context, rtaControllerNames[0], emptyArgsList);
            IPExpr controllerCheckCondition = new BinOpExpr(context, BinOpType.Eq, controllerCheckLhs, controllerCheckRhs);

            // It is guaranteed by the parser rule that at least two controllers exist.
            method.AddCallee(rtaControllers[0]);
            method.AddCallee(rtaControllers[1]);

            List<IPStmt> thenStatements = new List<IPStmt>();
            thenStatements.Add(new FunCallStmt(context, rtaControllers[0], emptyArgsList));
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[0]])));

            List<IPStmt> elseStatements = new List<IPStmt>();
            elseStatements.Add(new FunCallStmt(context, rtaControllers[1], emptyArgsList));
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[1]])));

            IPStmt controllerCheck = new IfStmt(context, controllerCheckCondition, new CompoundStmt(context, thenStatements), new CompoundStmt(context, elseStatements));

            for (int i = 2; i < rtaControllers.Count; i++)
            {
                controllerCheckRhs = new StringExpr(context, rtaControllerNames[i], emptyArgsList);
                controllerCheckCondition = new BinOpExpr(context, BinOpType.Eq, controllerCheckLhs, controllerCheckRhs);
                method.AddCallee(rtaControllers[i]);
                thenStatements.Clear();
                thenStatements.Add(new FunCallStmt(context, rtaControllers[i], emptyArgsList));
                thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[i]])));
                controllerCheck = new IfStmt(context, controllerCheckCondition, new CompoundStmt(context, thenStatements), controllerCheck);
            }

            isStartedCheckThenStatements.Add(controllerCheck);
            isStartedCheckThenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1)));

            IPExpr isStartedCheckCondition = new UnaryOpExpr(context, UnaryOpType.Not, new VariableAccessExpr(context, isStarted));
            IPStmt isStartedCheckThenBody = new CompoundStmt(context, isStartedCheckThenStatements);

            IPExpr checkDecisionPeriod = new BinOpExpr(context, BinOpType.Eq, new VariableAccessExpr(context, decisionPeriod), new VariableAccessExpr(context, decisionPeriodCount));
            thenStatements.Clear();
            thenStatements.Add(modeAssignment);
            thenStatements.Add(controllerCheck);
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1)));

            elseStatements.Clear();
            elseStatements.Add(controllerCheck);
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new BinOpExpr(context, BinOpType.Add, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1))));

            IPStmt isStartedElseBody = new IfStmt(context, checkDecisionPeriod, new CompoundStmt(context, thenStatements), new CompoundStmt(context, elseStatements));
            IPStmt isStartedIfStmt = new IfStmt(context, isStartedCheckCondition, isStartedCheckThenBody, isStartedElseBody);


            if (method.RTAEventHandler != null)
            {
                List<IPExpr> eventHandlerArgsList = new List<IPExpr>();
                if (method.Signature.Parameters.Count > 0)
                {
                    eventHandlerArgsList.Add(new VariableAccessExpr(context, method.Signature.Parameters[0]));
                }
                statements.Add(new FunCallStmt(context, method.RTAEventHandler, eventHandlerArgsList));
            }

            statements.Add(isStartedIfStmt);
            method.Body = new CompoundStmt(context, statements);
            return null;
        }

        public override object VisitTimeDrivenRTAModule(PParser.TimeDrivenRTAModuleContext context)
        {
            List<IPStmt> statements = new List<IPStmt>();
            if (method.Role.HasFlag(FunctionRole.EntryHandler) && machine.isStartedVar != null)
            {
                statements.Add(new AssignStmt(context, new VariableAccessExpr(context, machine.isStartedVar), new BoolLiteralExpr(context, false)));
                statements.Add(new AssignStmt(context, new VariableAccessExpr(context, machine.decisionPeriodCountVar), new IntLiteralExpr(context, 0)));
                method.Body = new CompoundStmt(context, statements);
                return null;
            }
            List<IPExpr> emptyArgsList = new List<IPExpr>();
            List<Function> rtaControllers = method.RTAControllers;
            List<string> rtaControllerNames = method.RTAControllerNames;
            Function rtaDecisionModule = method.RTADecisionModule;
            Dictionary<string, int> rtaDecisionPeriods = method.RTADecisionPeriods;
            Dictionary<string, Tuple<int, int>> rtaControllerPeriods = method.RTAControllerPeriods;

            Variable isStarted = machine.isStartedVar;

            List<IPStmt> isStartedCheckThenStatements = new List<IPStmt>();
            IPStmt isStartedTrueAssignment = new AssignStmt(context, new VariableAccessExpr(context, isStarted), new BoolLiteralExpr(context, true));
            isStartedCheckThenStatements.Add(isStartedTrueAssignment);

            method.AddCallee(rtaDecisionModule);
            IPExpr decisionModuleCall = new FunCallExpr(context, rtaDecisionModule, emptyArgsList);

            Variable mode = machine.modeVar;
            AssignStmt modeAssignment = new AssignStmt(context, new VariableAccessExpr(context, mode), decisionModuleCall);
            isStartedCheckThenStatements.Add(modeAssignment);

            Variable decisionPeriod = machine.decisionPeriodVar;

            Variable decisionPeriodCount = machine.decisionPeriodCountVar;

            Variable period = machine.periodVar;

            Variable periodUnit = machine.periodUnitVar;

            IPExpr controllerCheckLhs = new VariableAccessExpr(context, mode);
            IPExpr controllerCheckRhs = new StringExpr(context, rtaControllerNames[0], emptyArgsList);
            IPExpr controllerCheckCondition = new BinOpExpr(context, BinOpType.Eq, controllerCheckLhs, controllerCheckRhs);

            // It is guaranteed by the parser rule that at least two controllers exist.
            method.AddCallee(rtaControllers[0]);
            method.AddCallee(rtaControllers[1]);

            List<IPStmt> thenStatements = new List<IPStmt>();
            thenStatements.Add(new FunCallStmt(context, rtaControllers[0], emptyArgsList));
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[0]])));
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, period), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[0]].Item1)));
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, periodUnit), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[0]].Item2)));

            List<IPStmt> elseStatements = new List<IPStmt>();
            elseStatements.Add(new FunCallStmt(context, rtaControllers[1], emptyArgsList));
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[1]])));
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, period), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[1]].Item1)));
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, periodUnit), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[1]].Item2)));

            IPStmt controllerCheck = new IfStmt(context, controllerCheckCondition, new CompoundStmt(context, thenStatements), new CompoundStmt(context, elseStatements));

            for (int i = 2; i < rtaControllers.Count; i++)
            {
                controllerCheckRhs = new StringExpr(context, rtaControllerNames[i], emptyArgsList);
                controllerCheckCondition = new BinOpExpr(context, BinOpType.Eq, controllerCheckLhs, controllerCheckRhs);
                method.AddCallee(rtaControllers[i]);
                thenStatements.Clear();
                thenStatements.Add(new FunCallStmt(context, rtaControllers[i], emptyArgsList));
                thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriod), new IntLiteralExpr(context, rtaDecisionPeriods[rtaControllerNames[i]])));
                thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, period), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[i]].Item1)));
                thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, periodUnit), new IntLiteralExpr(context, rtaControllerPeriods[rtaControllerNames[i]].Item2)));
                controllerCheck = new IfStmt(context, controllerCheckCondition, new CompoundStmt(context, thenStatements), controllerCheck);
            }

            isStartedCheckThenStatements.Add(controllerCheck);
            isStartedCheckThenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1)));

            IPExpr isStartedCheckCondition = new UnaryOpExpr(context, UnaryOpType.Not, new VariableAccessExpr(context, isStarted));
            IPStmt isStartedCheckThenBody = new CompoundStmt(context, isStartedCheckThenStatements);

            IPExpr checkDecisionPeriod = new BinOpExpr(context, BinOpType.Eq, new VariableAccessExpr(context, decisionPeriod), new VariableAccessExpr(context, decisionPeriodCount));
            thenStatements.Clear();
            thenStatements.Add(modeAssignment);
            thenStatements.Add(controllerCheck);
            thenStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1)));

            elseStatements.Clear();
            elseStatements.Add(controllerCheck);
            elseStatements.Add(new AssignStmt(context, new VariableAccessExpr(context, decisionPeriodCount), new BinOpExpr(context, BinOpType.Add, new VariableAccessExpr(context, decisionPeriodCount), new IntLiteralExpr(context, 1))));

            IPStmt isStartedElseBody = new IfStmt(context, checkDecisionPeriod, new CompoundStmt(context, thenStatements), new CompoundStmt(context, elseStatements));
            IPStmt isStartedIfStmt = new IfStmt(context, isStartedCheckCondition, isStartedCheckThenBody, isStartedElseBody);

            statements.Add(isStartedIfStmt);
            method.Body = new CompoundStmt(context, statements);
            return null;
        }

        public override object VisitFunctionBody(PParser.FunctionBodyContext context)
        {
            // TODO: check that parameters have been added to internal scope?

            // Add all local variables to scope.
            foreach (PParser.VarDeclContext varDeclContext in context.varDecl())
            {
                Visit(varDeclContext);
            }

            // Build the statement trees
            StatementVisitor statementVisitor = new StatementVisitor(handler, machine, method);
            method.Body = (CompoundStmt)statementVisitor.Visit(context);
            return null;
        }

        public override object VisitVarDecl(PParser.VarDeclContext context)
        {
            foreach (PParser.IdenContext varName in context.idenList()._names)
            {
                Variable variable = method.Scope.Put(varName.GetText(), varName, VariableRole.Local);
                variable.Type = TypeResolver.ResolveType(context.type(), method.Scope, handler);
                method.AddLocalVariable(variable);
            }

            return null;
        }
    }
}